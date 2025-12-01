import * as vscode from 'vscode';
import { ColconService } from '../services/colconService';
import { getWorkspaceRoot, isValidROS2Workspace, getPackages } from '../utils/workspaceUtils';

export class BuildWizard {
    constructor(private colconService: ColconService) { }

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

        // Step 1: Select Scope
        const scope = await vscode.window.showQuickPick(
            [
                { label: 'Build All Packages', description: 'Build the entire workspace', value: 'all' },
                { label: 'Select Packages', description: 'Choose specific packages to build', value: 'select' }
            ],
            {
                placeHolder: 'Select build scope'
            }
        );

        if (!scope) {
            return;
        }

        let selectedPackages: string[] = [];

        if (scope.value === 'select') {
            const packages = getPackages(workspaceRoot);
            if (packages.length === 0) {
                vscode.window.showWarningMessage('No packages found in workspace.');
                return;
            }

            const packageSelection = await vscode.window.showQuickPick(
                packages.map(p => ({ label: p })),
                {
                    placeHolder: 'Select packages to build',
                    canPickMany: true
                }
            );

            if (!packageSelection || packageSelection.length === 0) {
                return;
            }

            selectedPackages = packageSelection.map(p => p.label);
        }

        // Step 2: Select Options
        const options = await vscode.window.showQuickPick(
            [
                { label: 'Symlink Install', description: 'Use --symlink-install', picked: true, value: 'symlink' }
            ],
            {
                placeHolder: 'Select build options',
                canPickMany: true
            }
        );

        if (!options) {
            return;
        }

        const useSymlink = options.some(o => o.value === 'symlink');

        // Execute
        try {
            await this.colconService.build(workspaceRoot, {
                symlinkInstall: useSymlink,
                packagesSelect: selectedPackages
            });

            vscode.window.showInformationMessage('Build completed successfully!');
        } catch (error) {
            vscode.window.showErrorMessage(`Build failed: ${error}`);
        }
    }
}
