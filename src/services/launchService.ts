import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';
import { EnvironmentService } from './environmentService';

export interface LaunchFile {
    name: string;
    path: string;
    package: string;
    type: 'python' | 'xml' | 'yaml';
}

export class LaunchService {
    constructor(
        private outputChannel: vscode.OutputChannel,
        private environmentService: EnvironmentService
    ) { }

    /**
     * Recursively finds all launch files in the workspace
     */
    public async findLaunchFiles(workspacePath: string): Promise<LaunchFile[]> {
        const srcPath = path.join(workspacePath, 'src');
        if (!fs.existsSync(srcPath)) {
            return [];
        }

        const launchFiles: LaunchFile[] = [];
        await this.scanDirectory(srcPath, launchFiles);
        return launchFiles;
    }

    private async scanDirectory(dir: string, launchFiles: LaunchFile[]): Promise<void> {
        const entries = await fs.promises.readdir(dir, { withFileTypes: true });

        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);

            if (entry.isDirectory()) {
                // Skip hidden directories and build artifacts
                if (entry.name.startsWith('.') || entry.name === 'build' || entry.name === 'install' || entry.name === 'log') {
                    continue;
                }
                await this.scanDirectory(fullPath, launchFiles);
            } else if (entry.isFile()) {
                if (entry.name.endsWith('.launch.py')) {
                    launchFiles.push(this.createLaunchFile(fullPath, 'python'));
                } else if (entry.name.endsWith('.launch.xml')) {
                    launchFiles.push(this.createLaunchFile(fullPath, 'xml'));
                } else if (entry.name.endsWith('.launch.yaml') || entry.name.endsWith('.launch.yml')) {
                    launchFiles.push(this.createLaunchFile(fullPath, 'yaml'));
                }
            }
        }
    }

    private createLaunchFile(filePath: string, type: 'python' | 'xml' | 'yaml'): LaunchFile {
        // Try to infer package name from path
        // Assuming structure: src/package_name/...
        const parts = filePath.split(path.sep);
        const srcIndex = parts.lastIndexOf('src');
        let packageName = 'unknown';

        if (srcIndex !== -1 && srcIndex + 1 < parts.length) {
            packageName = parts[srcIndex + 1];
        }

        return {
            name: path.basename(filePath),
            path: filePath,
            package: packageName,
            type
        };
    }

    /**
     * Runs a launch file in a terminal
     */
    public runLaunchFile(launchFile: LaunchFile): void {
        const envInfo = this.environmentService.detectEnvironment();

        if (!envInfo.rosDistro) {
            vscode.window.showErrorMessage('ROS 2 not detected. Cannot run launch file.');
            return;
        }

        const terminal = vscode.window.createTerminal(`ROS 2 Launch: ${launchFile.name}`);
        terminal.show();

        // Source environment
        terminal.sendText(`source /opt/ros/${envInfo.rosDistro}/setup.bash`);
        if (envInfo.isWorkspace && envInfo.hasInstall) {
            terminal.sendText(`source ${path.join(envInfo.workspacePath!, 'install', 'setup.bash')}`);
        }

        // Run launch command
        // If we know the package, use "ros2 launch package file"
        // Otherwise, use "ros2 launch path/to/file"
        if (launchFile.package !== 'unknown') {
            terminal.sendText(`ros2 launch ${launchFile.package} ${launchFile.name}`);
        } else {
            terminal.sendText(`ros2 launch ${launchFile.path}`);
        }

        this.outputChannel.appendLine(`Launched: ${launchFile.name} (${launchFile.path})`);
    }
}
