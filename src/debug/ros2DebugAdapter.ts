import * as vscode from 'vscode';
import { WorkspaceFolder, DebugConfiguration, ProviderResult, CancellationToken } from 'vscode';
import * as path from 'path';

export class Ros2DebugConfigurationProvider implements vscode.DebugConfigurationProvider {
    resolveDebugConfiguration(folder: WorkspaceFolder | undefined, config: DebugConfiguration, token?: CancellationToken): ProviderResult<DebugConfiguration> {
        // if launch.json is missing or empty
        if (!config.type && !config.request && !config.name) {
            const editor = vscode.window.activeTextEditor;
            if (editor && editor.document.languageId === 'python' && editor.document.fileName.endsWith('.launch.py')) {
                config.type = 'ros2';
                config.name = 'Launch Current File';
                config.request = 'launch';
                config.target = 'launch-file';
                config.launchFile = '${file}';
            }
        }

        // Validate based on target
        if (config.target === 'launch-file' && !config.launchFile) {
            return vscode.window.showErrorMessage("Cannot find a launch file to run. Please specify 'launchFile' in launch.json.").then(_ => {
                return undefined; // abort
            });
        }

        if (config.target === 'node' && (!config.package || !config.executable)) {
            return vscode.window.showErrorMessage("Cannot run node. Please specify 'package' and 'executable' in launch.json.").then(_ => {
                return undefined; // abort
            });
        }

        return config;
    }
}

export class Ros2DebugAdapterDescriptorFactory implements vscode.DebugAdapterDescriptorFactory {
    createDebugAdapterDescriptor(session: vscode.DebugSession, executable: vscode.DebugAdapterExecutable | undefined): ProviderResult<vscode.DebugAdapterDescriptor> {
        // We don't actually need a real debug adapter for now since we're just running commands
        // But VS Code expects one. We can use a simple inline implementation or just run the command and terminate.

        // For this phase, we'll just execute the command in a terminal and not actually attach a debugger.
        // This is a common pattern for "Run" extensions.

        this.runSession(session);

        // Return undefined to let VS Code know we're done (or maybe we need a dummy adapter?)
        // Actually, if we return undefined, it uses the default.
        // We should probably return a dummy adapter or handle it entirely here.

        return new vscode.DebugAdapterInlineImplementation(new Ros2DebugSession());
    }

    private runSession(session: vscode.DebugSession) {
        const config = session.configuration;
        const terminal = vscode.window.createTerminal(`ROS 2: ${config.name}`);
        terminal.show();

        // Source ROS 2 (system)
        const rosDistro = process.env.ROS_DISTRO || 'humble';
        terminal.sendText(`source /opt/ros/${rosDistro}/setup.bash`);

        // Source workspace or custom file
        if (config.sourceFile) {
            // Handle ${workspaceFolder} variable
            const sourceFile = config.sourceFile.replace('${workspaceFolder}', session.workspaceFolder?.uri.fsPath || '');
            terminal.sendText(`source ${sourceFile}`);
        } else if (session.workspaceFolder) {
            // Default fallback
            terminal.sendText(`source ${path.join(session.workspaceFolder.uri.fsPath, 'install', 'setup.bash')}`);
        }

        if (config.target === 'launch-file' || (!config.target && config.launchFile)) {
            terminal.sendText(`ros2 launch ${config.launchFile}`);
        } else if (config.target === 'node' || (!config.target && config.package && config.executable)) {
            terminal.sendText(`ros2 run ${config.package} ${config.executable}`);
        } else {
            terminal.sendText(`echo "Error: Invalid configuration. Missing launchFile or package/executable."`);
        }
    }
}

/**
 * A minimal debug adapter that does nothing but satisfy the interface.
 * Since we are running commands in the terminal, we don't need actual debugging protocol.
 */
class Ros2DebugSession implements vscode.DebugAdapter {
    private _sendMessage = new vscode.EventEmitter<vscode.DebugProtocolMessage>();
    readonly onDidSendMessage: vscode.Event<vscode.DebugProtocolMessage> = this._sendMessage.event;

    handleMessage(message: vscode.DebugProtocolMessage): void {
        // We don't need to handle any messages for this simple runner
    }

    dispose() {
        this._sendMessage.dispose();
    }
}
