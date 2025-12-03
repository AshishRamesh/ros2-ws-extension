
import * as fs from 'fs';
import * as path from 'path';

export interface DiscoveredNode {
    package: string;
    name: string;
    type: 'cpp' | 'python';
}

export class NodeDiscoveryService {
    /**
     * Finds all ROS 2 nodes in the workspace by parsing build files
     */
    public async findNodes(workspacePath: string): Promise<DiscoveredNode[]> {
        const srcPath = path.join(workspacePath, 'src');
        if (!fs.existsSync(srcPath)) {
            return [];
        }

        const nodes: DiscoveredNode[] = [];
        await this.scanDirectory(srcPath, nodes);
        return nodes;
    }

    private async scanDirectory(dir: string, nodes: DiscoveredNode[]): Promise<void> {
        const entries = await fs.promises.readdir(dir, { withFileTypes: true });

        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);

            if (entry.isDirectory()) {
                // Check if this directory is a package
                if (fs.existsSync(path.join(fullPath, 'package.xml'))) {
                    await this.scanPackage(fullPath, entry.name, nodes);
                } else {
                    // Recurse
                    if (!entry.name.startsWith('.') && entry.name !== 'build' && entry.name !== 'install' && entry.name !== 'log') {
                        await this.scanDirectory(fullPath, nodes);
                    }
                }
            }
        }
    }

    private async scanPackage(packagePath: string, packageName: string, nodes: DiscoveredNode[]): Promise<void> {
        // Check for CMakeLists.txt (C++)
        const cmakePath = path.join(packagePath, 'CMakeLists.txt');
        if (fs.existsSync(cmakePath)) {
            await this.scanCMakeLists(cmakePath, packageName, nodes);
        }

        // Check for setup.py (Python)
        const setupPyPath = path.join(packagePath, 'setup.py');
        if (fs.existsSync(setupPyPath)) {
            await this.scanSetupPy(setupPyPath, packageName, nodes);
        }
    }

    private async scanCMakeLists(cmakePath: string, packageName: string, nodes: DiscoveredNode[]): Promise<void> {
        try {
            const content = await fs.promises.readFile(cmakePath, 'utf-8');

            // Remove comments to avoid false matches
            const cleanedContent = content.replace(/#.*$/gm, '');

            // Method 1: Find add_executable(node_name ...)
            // Handles both single-line and multi-line declarations
            const executableRegex = /add_executable\s*\(\s*([a-zA-Z0-9_]+)/gi;
            let match;
            while ((match = executableRegex.exec(cleanedContent)) !== null) {
                const nodeName = match[1];
                // Skip if it looks like a variable (starts with ${)
                if (!nodeName.startsWith('${')) {

                    nodes.push({
                        package: packageName,
                        name: nodeName,
                        type: 'cpp'
                    });
                }
            }

            // Method 2: Find install(TARGETS ...) which lists executables to install
            // This catches executables that might be defined differently
            const installTargetsRegex = /install\s*\(\s*TARGETS\s+([\s\S]*?)(?:DESTINATION|RUNTIME|LIBRARY|ARCHIVE)/gi;
            while ((match = installTargetsRegex.exec(cleanedContent)) !== null) {
                const targetsBlock = match[1];
                // Split by whitespace and newlines
                const targets = targetsBlock.trim().split(/\s+/);

                for (const target of targets) {
                    // Clean up the target name
                    const cleanTarget = target.trim();

                    // Skip if empty, a variable, or a CMake keyword
                    if (cleanTarget &&
                        !cleanTarget.startsWith('$') &&
                        !cleanTarget.startsWith('#') &&
                        cleanTarget !== 'ARCHIVE' &&
                        cleanTarget !== 'LIBRARY' &&
                        cleanTarget !== 'RUNTIME' &&
                        cleanTarget !== 'DESTINATION' &&
                        /^[a-zA-Z0-9_]+$/.test(cleanTarget)) {

                        // Check if already added
                        const alreadyAdded = nodes.some(n => n.package === packageName && n.name === cleanTarget);
                        if (!alreadyAdded) {

                            nodes.push({
                                package: packageName,
                                name: cleanTarget,
                                type: 'cpp'
                            });
                        }
                    }
                }
            }

            // Method 3: Find install(PROGRAMS ...) for Python scripts in CMake packages
            const installProgramsRegex = /install\s*\(\s*PROGRAMS\s+([\s\S]*?)(?:DESTINATION|TYPE)/gi;
            while ((match = installProgramsRegex.exec(cleanedContent)) !== null) {
                const programsBlock = match[1];
                const programs = programsBlock.trim().split(/\s+/);

                for (const program of programs) {
                    const cleanProgram = program.trim();
                    if (cleanProgram && !cleanProgram.startsWith('$') && !cleanProgram.startsWith('#')) {
                        // Extract just the filename
                        const filename = path.basename(cleanProgram);
                        // User requested to keep .py extension for CMake installed programs
                        const nodeName = filename;

                        // Validate it's a reasonable node name (allow dots for .py)
                        if (/^[a-zA-Z0-9_.]+$/.test(nodeName)) {
                            const alreadyAdded = nodes.some(n => n.package === packageName && n.name === nodeName);
                            if (!alreadyAdded) {

                                nodes.push({
                                    package: packageName,
                                    name: nodeName,
                                    type: 'python'
                                });
                            }
                        }
                    }
                }
            }


        } catch (error) {
            console.error(`Failed to parse CMakeLists.txt at ${cmakePath}: ${error}`);
        }
    }

    private async scanSetupPy(setupPyPath: string, packageName: string, nodes: DiscoveredNode[]): Promise<void> {
        try {
            const content = await fs.promises.readFile(setupPyPath, 'utf-8');

            // Regex for console_scripts entry points: 'node_name = ...'
            // Matches: 'node_name = package.node:main' inside console_scripts
            const consoleScriptsRegex = /'console_scripts':\s*\[([\s\S]*?)\]/;
            const match = content.match(consoleScriptsRegex);

            if (match) {
                const scriptsBlock = match[1];
                const entryRegex = /'([a-zA-Z0-9_]+)\s*=\s*[^']+'/g;
                let entryMatch;
                while ((entryMatch = entryRegex.exec(scriptsBlock)) !== null) {
                    nodes.push({
                        package: packageName,
                        name: entryMatch[1],
                        type: 'python'
                    });
                }
            }

            // Also check for scripts/ folder if ament_cmake (hybrid)
            // But usually those are installed via install(PROGRAMS ...) which is hard to parse reliably from CMake
            // However, if we see scripts/ folder, we can guess
            const scriptsPath = path.join(path.dirname(setupPyPath), 'scripts');
            if (fs.existsSync(scriptsPath)) {
                const scriptEntries = await fs.promises.readdir(scriptsPath);
                for (const script of scriptEntries) {
                    if (script.endsWith('.py')) {
                        nodes.push({
                            package: packageName,
                            name: script.replace('.py', ''),
                            type: 'python'
                        });
                    }
                }
            }

        } catch (error) {
            console.error(`Failed to parse setup.py at ${setupPyPath}: ${error}`);
        }
    }
}
