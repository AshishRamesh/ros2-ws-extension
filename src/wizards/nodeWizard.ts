import * as vscode from 'vscode';
import { getWorkspaceRoot, isValidROS2Workspace, getPackages, isValidNodeName } from '../utils/workspaceUtils';
import { NodeGenerator } from '../generators/nodeGenerator';

export class NodeWizard {
    async run(): Promise<void> {
        // Validate workspace
        const workspaceRoot = getWorkspaceRoot();
        if (!workspaceRoot) {
            vscode.window.showErrorMessage('No workspace folder open');
            return;
        }

        if (!isValidROS2Workspace(workspaceRoot)) {
            vscode.window.showErrorMessage('Invalid ROS 2 workspace. Please ensure a src/ directory exists.');
            return;
        }

        // Step 1: Select package
        const packages = getPackages(workspaceRoot);
        if (packages.length === 0) {
            vscode.window.showErrorMessage('No packages found in workspace. Please create a package first.');
            return;
        }

        const selectedPackage = await vscode.window.showQuickPick(packages, {
            placeHolder: 'Select a package'
        });

        if (!selectedPackage) {
            return;
        }

        // Step 2: Get node name
        const nodeName = await vscode.window.showInputBox({
            prompt: 'Enter node name',
            placeHolder: 'my_node',
            validateInput: (value) => {
                if (!value) {
                    return 'Node name is required';
                }
                if (!isValidNodeName(value)) {
                    return 'Node name must be a valid identifier (letters, numbers, underscores)';
                }
                return null;
            }
        });

        if (!nodeName) {
            return;
        }

        // Step 3: Select language
        const language = await vscode.window.showQuickPick(
            [
                { label: 'C++', value: 'cpp' },
                { label: 'Python', value: 'python' }
            ],
            {
                placeHolder: 'Select programming language'
            }
        );

        if (!language) {
            return;
        }

        // Generate node
        try {
            const generator = new NodeGenerator();
            await generator.generateNode(
                workspaceRoot,
                selectedPackage,
                nodeName,
                language.value as 'cpp' | 'python'
            );

            vscode.window.showInformationMessage(`Node '${nodeName}' created successfully in package '${selectedPackage}'!`);
        } catch (error) {
            vscode.window.showErrorMessage(`Failed to create node: ${error}`);
        }
    }
}
