import * as vscode from 'vscode';
import { WelcomePanel } from './views/welcomePanel';
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
import { TopicEchoPanel } from './views/topicEchoPanel';
import { WorkflowConfigPanel } from './views/workflowConfigPanel';
import { WorkflowViewProvider } from './views/workflowViewProvider';
import { WorkflowRunner } from './services/workflowRunner';
import { Workflow, WorkflowConfiguration } from './types/workflow';

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
	const workflowRunner = new WorkflowRunner(launchService, nodeDiscoveryService);

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

	// Run Node - directly execute a node
	const runNodeCmd = vscode.commands.registerCommand('ros2.runNode', async (node: any) => {
		if (!node || !node.package || !node.name) {
			vscode.window.showErrorMessage('Invalid node selected');
			return;
		}

		const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
		if (!workspaceRoot) {
			vscode.window.showErrorMessage('No workspace folder open');
			return;
		}

		const terminalName = `ROS 2: ${node.package}/${node.name}`;

		// Check if terminal already exists
		let terminal = vscode.window.terminals.find(t => t.name === terminalName);

		if (!terminal) {
			// Create new terminal if it doesn't exist
			terminal = vscode.window.createTerminal(terminalName);
		}

		terminal.show();

		// Source workspace setup
		const setupScript = `${workspaceRoot}/install/setup.bash`;
		terminal.sendText(`source ${setupScript}`);

		// Run the node
		terminal.sendText(`ros2 run ${node.package} ${node.name}`);
	});

	// Workflow Configuration
	const configureWorkflowCmd = vscode.commands.registerCommand('ros2.configureWorkflow', () => {
		WorkflowConfigPanel.createOrShow(
			context.extensionUri,
			(config: WorkflowConfiguration) => {
				// Save to workspace settings
				const workspaceConfig = vscode.workspace.getConfiguration('ros2Toolkit');
				workspaceConfig.update('workflowSteps', config, vscode.ConfigurationTarget.Workspace);
			},
			launchService,
			nodeDiscoveryService
		);
	});

	// Run Workflow
	const runWorkflowCmd = vscode.commands.registerCommand('ros2.runWorkflow', async (workflow?: Workflow) => {
		const config = vscode.workspace.getConfiguration('ros2Toolkit');
		const workflowConfig: WorkflowConfiguration = config.get('workflowSteps') || { workflows: [] };

		let targetWorkflow = workflow;

		// If no workflow passed, try to find one (maybe prompt user or check for single workflow)
		if (!targetWorkflow) {
			// If there's only one workflow, run it
			if (workflowConfig.workflows.length === 1) {
				targetWorkflow = workflowConfig.workflows[0];
			} else if (workflowConfig.workflows.length > 1) {
				// Show quick pick
				const selected = await vscode.window.showQuickPick(
					workflowConfig.workflows.map(w => w.name),
					{ placeHolder: 'Select workflow to run' }
				);
				if (selected) {
					targetWorkflow = workflowConfig.workflows.find(w => w.name === selected);
				}
			}
		}

		if (!targetWorkflow) {
			if (workflowConfig.workflows.length === 0) {
				const configure = await vscode.window.showInformationMessage(
					'No workflows configured. Configure one now?',
					'Configure', 'Cancel'
				);
				if (configure === 'Configure') {
					vscode.commands.executeCommand('ros2.configureWorkflow');
				}
			}
			return;
		}

		await workflowRunner.runWorkflow(targetWorkflow);
	});

	// Stop Workflow
	const stopWorkflowCmd = vscode.commands.registerCommand('ros2.stopWorkflow', () => {
		workflowRunner.stopWorkflow();
		vscode.window.showInformationMessage('Workflow stopped');
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

	// Refresh nodes and launch files after build completes
	colconService.setOnBuildComplete(() => {
		console.log('Build completed - refreshing nodes and launch files');
		runDebugProvider.refresh();
	});

	const topicsTreeProvider = new TopicsTreeViewProvider(topicsService, echoService);
	const topicsView = vscode.window.createTreeView('ros2Topics', {
		treeDataProvider: topicsTreeProvider
	});

	const bagProvider = new BagViewProvider();
	const bagView = vscode.window.createTreeView('ros2Bag', {
		treeDataProvider: bagProvider
	});

	const workflowProvider = new WorkflowViewProvider(workflowRunner);
	const workflowView = vscode.window.createTreeView('ros2Workflows', {
		treeDataProvider: workflowProvider
	});

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
		runNodeCmd,
		configureWorkflowCmd,
		runWorkflowCmd,
		stopWorkflowCmd,
		workspaceView,
		runDebugView,
		topicsView,
		bagView,
		workflowView
	);
}

export function deactivate() {
	console.log('ROS 2 Developer Tools extension is now deactivated');
}
