import * as vscode from 'vscode';
import { WelcomePanel } from './views/welcomePanel';
import { ROS2SidebarProvider } from './views/sidebarProvider';
import { WorkspaceViewProvider } from './views/workspaceViewProvider';
import { RunDebugViewProvider } from './views/runDebugViewProvider';
import { TopicsTreeViewProvider } from './views/topicsTreeViewProvider';
import { BagViewProvider } from './views/bagViewProvider';
import { ColconService } from './services/colconService';
import { PackageWizard } from './wizards/packageWizard';
import { NodeWizard } from './wizards/nodeWizard';
import { BuildWizard } from './wizards/buildWizard';
import { EnvironmentService } from './services/environmentService';
import { LaunchService } from './services/launchService';
import { Ros2DebugConfigurationProvider, Ros2DebugAdapterDescriptorFactory } from './debug/ros2DebugAdapter';
import { NodeDiscoveryService } from './services/nodeDiscoveryService';
import { RunConfigurationWizard } from './wizards/runConfigurationWizard';
import { BagRecordingWizard } from './wizards/bagRecordingWizard';
import { BagPlaybackWizard } from './wizards/bagPlaybackWizard';
import { TopicsService } from './services/topicsService';
import { EchoService } from './services/echoService';
import { TopicsViewProvider } from './views/topicsViewProvider';
import { TopicEchoPanel } from './views/topicEchoPanel';

let outputChannel: vscode.OutputChannel;

export function activate(context: vscode.ExtensionContext) {
	console.log('ROS 2 Developer Tools extension is now active!');

	// Create output channel for build logs
	outputChannel = vscode.window.createOutputChannel('ROS 2 Build');
	context.subscriptions.push(outputChannel);

	// Initialize services
	const colconService = new ColconService(outputChannel);
	const environmentService = new EnvironmentService(context);
	const launchService = new LaunchService(outputChannel, environmentService);
	const nodeDiscoveryService = new NodeDiscoveryService();
	const topicsService = new TopicsService();
	const echoService = new EchoService();

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
		const wizard = new BuildWizard(colconService);
		await wizard.run();
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
		WelcomePanel.createOrShow(context.extensionUri, environmentService);
	});

	// Setup Environment
	const setupEnvCmd = vscode.commands.registerCommand('ros2.setupEnvironment', () => {
		environmentService.setupEnvironment();
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

	// Generate .gitignore
	const generateGitignoreCmd = vscode.commands.registerCommand('ros2.generateGitignore', async () => {
		const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
		if (!workspaceFolder) {
			vscode.window.showErrorMessage('No workspace folder open');
			return;
		}
		await colconService.generateGitignore(workspaceFolder.uri.fsPath);
	});

	// Run Launch File
	const runLaunchFileCmd = vscode.commands.registerCommand('ros2.runLaunchFile', (launchFile) => {
		launchService.runLaunchFile(launchFile);
	});

	// Create Run Configuration
	const createRunConfigCmd = vscode.commands.registerCommand('ros2.createRunConfiguration', async (node?: any) => {
		// If called from tree view, node is passed
		// If called from palette, node is undefined
		const wizard = new RunConfigurationWizard(nodeDiscoveryService);
		// TODO: Pass node to wizard if available to skip selection
		await wizard.run();
	});

	// Echo Topic
	const echoTopicCmd = vscode.commands.registerCommand('ros2.echoTopic', async (topic: string) => {
		const terminal = vscode.window.createTerminal(`ROS 2 Echo: ${topic}`);
		terminal.show();
		terminal.sendText(`ros2 topic echo ${topic}`);
	});

	// Refresh Topics
	const refreshTopicsCmd = vscode.commands.registerCommand('ros2.refreshTopics', () => {
		topicsTreeProvider.refresh();
		vscode.window.showInformationMessage('Topics refreshed');
	});

	// View Topic Messages
	const viewTopicMessagesCmd = vscode.commands.registerCommand('ros2.viewTopicMessages', (topic: string) => {
		TopicEchoPanel.createOrShow(context.extensionUri, topic, echoService);
	});

	// Record Bag
	const recordBagCmd = vscode.commands.registerCommand('ros2.recordBag', async () => {
		const wizard = new BagRecordingWizard(topicsService);
		await wizard.run();
	});

	// Play Bag
	const playBagCmd = vscode.commands.registerCommand('ros2.playBag', async () => {
		const wizard = new BagPlaybackWizard();
		await wizard.run();
	});

	// Register view providers
	const workspaceProvider = new WorkspaceViewProvider();
	const workspaceView = vscode.window.createTreeView('ros2Workspace', {
		treeDataProvider: workspaceProvider
	});

	const runDebugProvider = new RunDebugViewProvider(launchService, nodeDiscoveryService);
	const runDebugView = vscode.window.createTreeView('ros2RunDebug', {
		treeDataProvider: runDebugProvider
	});

	const topicsTreeProvider = new TopicsTreeViewProvider(topicsService, echoService);
	const topicsView = vscode.window.createTreeView('ros2Topics', {
		treeDataProvider: topicsTreeProvider
	});

	const bagProvider = new BagViewProvider();
	const bagView = vscode.window.createTreeView('ros2Bag', {
		treeDataProvider: bagProvider
	});

	// Register Topics View Provider
	const topicsViewProvider = new TopicsViewProvider(context.extensionUri, topicsService, echoService);
	context.subscriptions.push(
		vscode.window.registerWebviewViewProvider(TopicsViewProvider.viewType, topicsViewProvider)
	);

	// Register Debug Provider
	const debugProvider = new Ros2DebugConfigurationProvider();
	context.subscriptions.push(vscode.debug.registerDebugConfigurationProvider('ros2', debugProvider));

	const debugFactory = new Ros2DebugAdapterDescriptorFactory();
	context.subscriptions.push(vscode.debug.registerDebugAdapterDescriptorFactory('ros2', debugFactory));

	// Add all subscriptions
	context.subscriptions.push(
		initWorkspaceCmd,
		buildCmd,
		cleanBuildCmd,
		openPanelCmd,
		createPackageCmd,
		createNodeCmd,
		generateGitignoreCmd,
		setupEnvCmd,
		runLaunchFileCmd,
		createRunConfigCmd,
		echoTopicCmd,
		refreshTopicsCmd,
		viewTopicMessagesCmd,
		recordBagCmd,
		playBagCmd,
		workspaceView,
		runDebugView,
		topicsView,
		bagView
	);
}

export function deactivate() {
	console.log('ROS 2 Developer Tools extension is now deactivated');
}
