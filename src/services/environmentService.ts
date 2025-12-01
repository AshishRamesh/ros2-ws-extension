import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

export interface Ros2EnvInfo {
    isWorkspace: boolean;
    rosDistro: string | null;
    hasInstall: boolean;
    workspacePath: string | null;
}

export class EnvironmentService {
    constructor(private context: vscode.ExtensionContext) { }

    /**
     * Detects the current ROS 2 environment status
     */
    public detectEnvironment(): Ros2EnvInfo {
        const workspaceFolders = vscode.workspace.workspaceFolders;
        const workspacePath = workspaceFolders ? workspaceFolders[0].uri.fsPath : null;

        // Check for ROS_DISTRO environment variable
        const rosDistro = process.env.ROS_DISTRO || null;

        if (!workspacePath) {
            return {
                isWorkspace: false,
                rosDistro,
                hasInstall: false,
                workspacePath: null
            };
        }

        // Check for src directory
        const srcPath = path.join(workspacePath, 'src');
        const isWorkspace = fs.existsSync(srcPath) && fs.statSync(srcPath).isDirectory();

        // Check for install/setup.bash
        const installSetupPath = path.join(workspacePath, 'install', 'setup.bash');
        const hasInstall = fs.existsSync(installSetupPath);

        return {
            isWorkspace,
            rosDistro,
            hasInstall,
            workspacePath
        };
    }

    /**
     * Sources ROS 2 and workspace setup files in a new terminal
     */
    public setupEnvironment(): void {
        const envInfo = this.detectEnvironment();

        if (!envInfo.rosDistro) {
            vscode.window.showErrorMessage('ROS_DISTRO environment variable not set. Is ROS 2 installed?');
            return;
        }

        const terminal = vscode.window.createTerminal('ROS 2 Environment');
        terminal.show();

        // Source ROS 2
        terminal.sendText(`source /opt/ros/${envInfo.rosDistro}/setup.bash`);

        // Source workspace if available
        if (envInfo.isWorkspace && envInfo.hasInstall) {
            terminal.sendText(`source ${path.join(envInfo.workspacePath!, 'install', 'setup.bash')}`);
        } else if (envInfo.isWorkspace) {
            terminal.sendText('# Workspace not built yet. Run "colcon build" first.');
        }
    }
}
