import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

export interface DiscoveredNode {
    package: string;
    name: string;
    type: 'cpp' | 'python';
    path?: string;
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
            // Regex to find add_executable(node_name ...)
            const regex = /add_executable\s*\(\s*([a-zA-Z0-9_]+)/g;
            let match;
            while ((match = regex.exec(content)) !== null) {
                nodes.push({
                    package: packageName,
                    name: match[1],
                    type: 'cpp'
                });
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
                            name: script.replace('.py', ''), // Usually run as node_name.py but ROS 2 run often strips extension or uses filename
                            type: 'python',
                            path: path.join(scriptsPath, script)
                        });
                    }
                }
            }

        } catch (error) {
            console.error(`Failed to parse setup.py at ${setupPyPath}: ${error}`);
        }
    }
}
