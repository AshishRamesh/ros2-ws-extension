import * as vscode from 'vscode';
import { getWorkspaceRoot, isValidROS2Workspace, isValidPackageName } from '../utils/workspaceUtils';
import { PackageGenerator } from '../generators/packageGenerator';

export class PackageWizard {
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

        // Step 1: Get package name
        const packageName = await vscode.window.showInputBox({
            prompt: 'Enter package name',
            placeHolder: 'my_package',
            validateInput: (value) => {
                if (!value) {
                    return 'Package name is required';
                }
                if (!isValidPackageName(value)) {
                    return 'Package name must start with a lowercase letter and contain only lowercase letters, numbers, and underscores';
                }
                return null;
            }
        });

        if (!packageName) {
            return;
        }

        // Step 2: Select build type
        const buildType = await vscode.window.showQuickPick(
            [
                { label: 'ament_cmake', description: 'C++ package' },
                { label: 'ament_python', description: 'Python package' }
            ],
            {
                placeHolder: 'Select build type'
            }
        );

        if (!buildType) {
            return;
        }

        // Step 3: Get dependencies
        const dependenciesInput = await vscode.window.showInputBox({
            prompt: 'Enter dependencies (comma-separated, optional)',
            placeHolder: 'rclcpp, std_msgs, geometry_msgs'
        });

        const dependencies = dependenciesInput
            ? dependenciesInput.split(',').map(d => d.trim()).filter(d => d.length > 0)
            : [];

        // Step 4: Get description
        const description = await vscode.window.showInputBox({
            prompt: 'Enter package description (optional)',
            placeHolder: 'A ROS 2 package'
        });

        // Generate package
        try {
            const generator = new PackageGenerator();
            await generator.generatePackage(
                workspaceRoot,
                packageName,
                buildType.label as 'ament_cmake' | 'ament_python',
                dependencies,
                description || 'A ROS 2 package'
            );

            vscode.window.showInformationMessage(`Package '${packageName}' created successfully!`);
        } catch (error) {
            vscode.window.showErrorMessage(`Failed to create package: ${error}`);
        }
    }
}
