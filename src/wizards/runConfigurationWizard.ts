import * as vscode from 'vscode';
import { NodeDiscoveryService, DiscoveredNode } from '../services/nodeDiscoveryService';
import { getWorkspaceRoot } from '../utils/workspaceUtils';

export class RunConfigurationWizard {
    constructor(private nodeDiscoveryService: NodeDiscoveryService) { }

    async run(): Promise<void> {
        const workspaceRoot = getWorkspaceRoot();
        if (!workspaceRoot) {
            vscode.window.showErrorMessage('No workspace folder open');
            return;
        }

        // Step 1: Discover Nodes
        const nodes = await this.nodeDiscoveryService.findNodes(workspaceRoot);
        if (nodes.length === 0) {
            vscode.window.showWarningMessage('No ROS 2 nodes found in workspace.');
            return;
        }

        // Step 2: Select Node
        const selectedNodeItem = await vscode.window.showQuickPick(
            nodes.map(n => ({
                label: n.name,
                description: `${n.package} (${n.type})`,
                node: n
            })),
            {
                placeHolder: 'Select a node to run'
            }
        );

        if (!selectedNodeItem) {
            return;
        }

        const node = selectedNodeItem.node;

        // Step 3: Configure Source Script (Optional)
        // Default to workspace install/setup.bash
        const defaultSource = '${workspaceFolder}/install/setup.bash';
        const sourceScript = await vscode.window.showInputBox({
            prompt: 'Enter path to setup file to source (optional)',
            value: defaultSource,
            placeHolder: '/opt/ros/humble/setup.bash'
        });

        if (sourceScript === undefined) {
            return; // User cancelled
        }

        // Step 4: Add to launch.json
        await this.addToLaunchConfig(node, sourceScript);
    }

    private async addToLaunchConfig(node: DiscoveredNode, sourceScript: string): Promise<void> {
        const launchConfig = vscode.workspace.getConfiguration('launch');
        const configurations = launchConfig.get<any[]>('configurations') || [];

        const newConfig = {
            type: 'ros2',
            request: 'launch',
            name: `ROS 2: ${node.name}`,
            target: 'node',
            package: node.package,
            executable: node.name,
            sourceFile: sourceScript
        };

        configurations.push(newConfig);

        await launchConfig.update('configurations', configurations, vscode.ConfigurationTarget.Workspace);

        const action = await vscode.window.showInformationMessage(
            `Added configuration for '${node.name}'.`,
            'Open launch.json',
            'Run Now'
        );

        if (action === 'Open launch.json') {
            const launchFile = vscode.workspace.workspaceFolders?.[0].uri.fsPath + '/.vscode/launch.json';
            const doc = await vscode.workspace.openTextDocument(launchFile);
            await vscode.window.showTextDocument(doc);
        } else if (action === 'Run Now') {
            await vscode.debug.startDebugging(vscode.workspace.workspaceFolders?.[0], newConfig);
        }
    }
}
