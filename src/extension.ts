import * as vscode from 'vscode';
import { WelcomePanel } from './views/welcomePanel';
import { ROS2SidebarProvider } from './views/sidebarProvider';
import { ColconService } from './services/colconService';
import { PackageWizard } from './wizards/packageWizard';
import { NodeWizard } from './wizards/nodeWizard';

let outputChannel: vscode.OutputChannel;

export function activate(context: vscode.ExtensionContext) {
	console.log('ROS 2 Developer Tools extension is now active!');

	// Create output channel for build logs
	outputChannel = vscode.window.createOutputChannel('ROS 2 Build');
	context.subscriptions.push(outputChannel);

	// Initialize services
	const colconService = new ColconService(outputChannel);

	// Register commands

	// Initialize Workspace
	const initWorkspaceCmd = vscode.commands.registerCommand('ros2.initWorkspace', async () => {
		const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
		if (!workspaceFolder) {
			vscode.window.showErrorMessage('No workspace folder open');
			return;
		}

		try {
			await colconService.initWorkspace(workspaceFolder.uri.fsPath);
			vscode.window.showInformationMessage('ROS 2 workspace initialized successfully!');
		} catch (error) {
			vscode.window.showErrorMessage(`Failed to initialize workspace: ${error}`);
		}
	});

	// Build Workspace
	const buildCmd = vscode.commands.registerCommand('ros2.build', async () => {
		const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
		if (!workspaceFolder) {
			vscode.window.showErrorMessage('No workspace folder open');
			return;
		}

		try {
			// Get build options from sidebar
			const options = {
				symlinkInstall: sidebarProvider.getSymlinkInstall(),
				packagesSelect: []
			};

			// Generate .gitignore if enabled
			if (sidebarProvider.getAddGitignore()) {
				await colconService.generateGitignore(workspaceFolder.uri.fsPath);
			}

			await colconService.build(workspaceFolder.uri.fsPath, options);
			vscode.window.showInformationMessage('Build completed successfully!');
		} catch (error) {
			vscode.window.showErrorMessage(`Build failed: ${error}`);
		}
	});

	// Clean Build
	const cleanBuildCmd = vscode.commands.registerCommand('ros2.cleanBuild', async () => {
		const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
		if (!workspaceFolder) {
			vscode.window.showErrorMessage('No workspace folder open');
			return;
		}

		try {
			await colconService.clean(workspaceFolder.uri.fsPath);
			vscode.window.showInformationMessage('Clean completed successfully!');
		} catch (error) {
			vscode.window.showErrorMessage(`Clean failed: ${error}`);
		}
	});

	// Open Welcome Panel
	const openPanelCmd = vscode.commands.registerCommand('ros2.openPanel', () => {
		WelcomePanel.createOrShow(context.extensionUri);
	});

	// Create Package
	const createPackageCmd = vscode.commands.registerCommand('ros2.createPackage', async () => {
		const wizard = new PackageWizard();
		await wizard.run();
	});

	// Create Node
	const createNodeCmd = vscode.commands.registerCommand('ros2.createNode', async () => {
		const wizard = new NodeWizard();
		await wizard.run();
	});

	// Toggle Symlink Install
	const toggleSymlinkCmd = vscode.commands.registerCommand('ros2.toggleSymlinkInstall', () => {
		sidebarProvider.toggleSymlinkInstall();
	});

	// Toggle Gitignore
	const toggleGitignoreCmd = vscode.commands.registerCommand('ros2.toggleGitignore', () => {
		sidebarProvider.toggleGitignore();
	});

	// Register sidebar provider
	const sidebarProvider = new ROS2SidebarProvider();
	const treeView = vscode.window.createTreeView('ros2ControlPanel', {
		treeDataProvider: sidebarProvider
	});

	// Add all subscriptions
	context.subscriptions.push(
		initWorkspaceCmd,
		buildCmd,
		cleanBuildCmd,
		openPanelCmd,
		createPackageCmd,
		createNodeCmd,
		toggleSymlinkCmd,
		toggleGitignoreCmd,
		treeView
	);
}

export function deactivate() {
	console.log('ROS 2 Developer Tools extension is now deactivated');
}
