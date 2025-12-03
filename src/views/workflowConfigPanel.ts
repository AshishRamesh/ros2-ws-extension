import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import { Workflow, WorkflowConfiguration } from '../types/workflow';
import { LaunchService } from '../services/launchService';
import { NodeDiscoveryService } from '../services/nodeDiscoveryService';

export class WorkflowConfigPanel {
    public static currentPanel: WorkflowConfigPanel | undefined;
    private readonly _panel: vscode.WebviewPanel;
    private readonly _extensionUri: vscode.Uri;
    private _disposables: vscode.Disposable[] = [];
    private readonly _onSave: (config: WorkflowConfiguration) => void;
    private readonly _launchService: LaunchService;
    private readonly _nodeDiscoveryService: NodeDiscoveryService;

    private constructor(
        panel: vscode.WebviewPanel,
        extensionUri: vscode.Uri,
        onSave: (config: WorkflowConfiguration) => void,
        launchService: LaunchService,
        nodeDiscoveryService: NodeDiscoveryService
    ) {
        this._panel = panel;
        this._extensionUri = extensionUri;
        this._onSave = onSave;
        this._launchService = launchService;
        this._nodeDiscoveryService = nodeDiscoveryService;

        this._panel.onDidDispose(() => this.dispose(), null, this._disposables);

        this._panel.webview.html = this._getHtmlForWebview(this._panel.webview);

        this._panel.webview.onDidReceiveMessage(
            async message => {
                switch (message.command) {
                    case 'getConfig':
                        this.sendConfig();
                        break;
                    case 'save':
                        await this.saveConfig(message.config);
                        break;
                }
            },
            null,
            this._disposables
        );
    }

    public static createOrShow(
        extensionUri: vscode.Uri,
        onSave: (config: WorkflowConfiguration) => void,
        launchService: LaunchService,
        nodeDiscoveryService: NodeDiscoveryService
    ) {
        const column = vscode.window.activeTextEditor
            ? vscode.window.activeTextEditor.viewColumn
            : undefined;

        if (WorkflowConfigPanel.currentPanel) {
            WorkflowConfigPanel.currentPanel._panel.reveal(column);
            return;
        }

        const panel = vscode.window.createWebviewPanel(
            'ros2WorkflowConfig',
            'ROS 2 Workflows',
            column || vscode.ViewColumn.One,
            {
                enableScripts: true,
                localResourceRoots: [vscode.Uri.joinPath(extensionUri, 'media')]
            }
        );

        WorkflowConfigPanel.currentPanel = new WorkflowConfigPanel(
            panel,
            extensionUri,
            onSave,
            launchService,
            nodeDiscoveryService
        );
    }

    public dispose() {
        WorkflowConfigPanel.currentPanel = undefined;

        this._panel.dispose();

        while (this._disposables.length) {
            const x = this._disposables.pop();
            if (x) {
                x.dispose();
            }
        }
    }

    private async sendConfig() {
        // Get config from workspace settings using the same key as extension.ts
        const config = vscode.workspace.getConfiguration('ros2Toolkit');
        const workflowConfig = config.get('workflowSteps') as WorkflowConfiguration || { workflows: [] };

        const nodes = await this.getAvailableNodes();
        const launchFiles = await this.getAvailableLaunchFiles();

        this._panel.webview.postMessage({
            command: 'loadConfig',
            config: workflowConfig,
        });

        this._panel.webview.postMessage({
            command: 'loadItems',
            nodes,
            launchFiles
        });
    }

    private async saveConfig(config: WorkflowConfiguration) {
        this._onSave(config);
        vscode.window.showInformationMessage('Workflow configuration saved!');
    }

    private async getAvailableNodes(): Promise<any[]> {
        const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
        if (!workspaceFolder) {
            return [];
        }
        return await this._nodeDiscoveryService.findNodes(workspaceFolder.uri.fsPath);
    }

    private async getAvailableLaunchFiles(): Promise<any[]> {
        const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
        if (!workspaceFolder) {
            return [];
        }
        return await this._launchService.findLaunchFiles(workspaceFolder.uri.fsPath);
    }

    private _getHtmlForWebview(webview: vscode.Webview) {
        // Icons
        const icons = {
            save: `<svg class="icon" viewBox="0 0 16 16" xmlns="http://www.w3.org/2000/svg"><path d="M11 1H3a2 2 0 0 0-2 2v10a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2V5l-4-4zm3 12a1 1 0 0 1-1 1H3a1 1 0 0 1-1-1V3a1 1 0 0 1 1-1h7.59L14 5.41V13zM8 10a2 2 0 1 0 0-4 2 2 0 0 0 0 4zm3-6V2H5v2h6z"/></svg>`,
            link: `<svg class="icon" viewBox="0 0 16 16" xmlns="http://www.w3.org/2000/svg"><path d="M6.354 5.5H4a3 3 0 0 0 0 6h3a3 3 0 0 0 2.83-4H9c-.086 0-.17.01-.25.031A2 2 0 0 1 7 10.5H4a2 2 0 1 1 0-4h1.535c.218-.376.495-.714.82-1z"/><path d="M9 5.5a3 3 0 0 0-2.83 4h1.098A2 2 0 0 1 9 6.5h3a2 2 0 1 1 0 4h-1.535a4.02 4.02 0 0 1-.82 1H12a3 3 0 1 0 0-6H9z"/></svg>`,
            trash: `<svg class="icon" viewBox="0 0 16 16" xmlns="http://www.w3.org/2000/svg"><path d="M5.5 5.5A.5.5 0 0 1 6 6v6a.5.5 0 0 1-1 0V6a.5.5 0 0 1 .5-.5zm2.5 0a.5.5 0 0 1 .5.5v6a.5.5 0 0 1-1 0V6a.5.5 0 0 1 .5-.5zm3 .5a.5.5 0 0 0-1 0v6a.5.5 0 0 0 1 0V6z"/><path fill-rule="evenodd" d="M14.5 3a1 1 0 0 1-1 1H13v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V4h-.5a1 1 0 0 1-1-1V2a1 1 0 0 1 1-1H6a1 1 0 0 1 1-1h2a1 1 0 0 1 1 1h3.5a1 1 0 0 1 1 1v1zM4.118 4 4 4.059V13a1 1 0 0 0 1 1h6a1 1 0 0 0 1-1V4.059L11.882 4H4.118zM2.5 3V2h11v1h-11z"/></svg>`,
            up: `<svg class="icon" viewBox="0 0 16 16" xmlns="http://www.w3.org/2000/svg"><path fill-rule="evenodd" d="M7.646 4.646a.5.5 0 0 1 .708 0l6 6a.5.5 0 0 1-.708.708L8 5.707l-5.646 5.647a.5.5 0 0 1-.708-.708l6-6z"/></svg>`,
            down: `<svg class="icon" viewBox="0 0 16 16" xmlns="http://www.w3.org/2000/svg"><path fill-rule="evenodd" d="M1.646 4.646a.5.5 0 0 1 .708 0L8 10.293l5.646-5.647a.5.5 0 0 1 .708.708l-6 6a.5.5 0 0 1-.708 0l-6-6a.5.5 0 0 1 0-.708z"/></svg>`
        };

        return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROS 2 Workflow Configuration</title>
    <style>
        body {
            font-family: var(--vscode-font-family);
            color: var(--vscode-foreground);
            background-color: var(--vscode-editor-background);
            padding: 20px;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
        }
        h1 {
            color: var(--vscode-titleBar-activeForeground);
            border-bottom: 1px solid var(--vscode-panel-border);
            padding-bottom: 10px;
        }
        .workflow-list {
            margin: 20px 0;
        }
        .workflow-item {
            background: var(--vscode-editor-inactiveSelectionBackground);
            border: 1px solid var(--vscode-panel-border);
            border-radius: 4px;
            padding: 15px;
            margin-bottom: 10px;
        }
        .workflow-item.active {
            border-color: var(--vscode-focusBorder);
            background: var(--vscode-list-activeSelectionBackground);
        }
        .step-list {
            margin: 10px 0;
        }
        .step-item {
            background: var(--vscode-input-background);
            border: 1px solid var(--vscode-input-border);
            border-radius: 3px;
            padding: 10px;
            margin: 5px 0;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .step-item.disabled {
            opacity: 0.5;
        }
        button {
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            padding: 6px 14px;
            border-radius: 2px;
            cursor: pointer;
            font-size: 13px;
            display: inline-flex;
            align-items: center;
            gap: 5px;
        }
        button:hover {
            background: var(--vscode-button-hoverBackground);
        }
        button.secondary {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
        }
        button.secondary:hover {
            background: var(--vscode-button-secondaryHoverBackground);
        }
        input, select {
            background: var(--vscode-input-background);
            color: var(--vscode-input-foreground);
            border: 1px solid var(--vscode-input-border);
            padding: 4px 8px;
            border-radius: 2px;
            font-size: 13px;
        }
        .form-group {
            margin: 10px 0;
        }
        .form-group label {
            display: block;
            margin-bottom: 5px;
            font-weight: 500;
        }
        .step-controls {
            display: flex;
            gap: 5px;
            margin-left: auto;
        }
        .step-type {
            font-weight: bold;
            color: var(--vscode-textLink-foreground);
            min-width: 80px;
            display: flex;
            align-items: center;
            gap: 5px;
        }
        .step-description {
            flex: 1;
        }
        .actions {
            margin-top: 20px;
            display: flex;
            gap: 10px;
        }
        .modal {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.5);
            justify-content: center;
            align-items: center;
        }
        .modal.show {
            display: flex;
        }
        .modal-content {
            background: var(--vscode-editor-background);
            border: 1px solid var(--vscode-panel-border);
            border-radius: 4px;
            padding: 20px;
            max-width: 500px;
            width: 90%;
        }
        .icon {
            width: 14px;
            height: 14px;
            fill: currentColor;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚀 ROS 2 Workflow Configuration</h1>
        
        <div class="actions">
            <button onclick="addWorkflow()">+ New Workflow</button>
            <button class="secondary" onclick="saveConfig()">${icons.save} Save Configuration</button>
        </div>

        <div id="workflowList" class="workflow-list"></div>
    </div>

    <!-- Add Workflow Modal -->
    <div id="addWorkflowModal" class="modal">
        <div class="modal-content">
            <h2>New Workflow</h2>
            <div class="form-group">
                <label>Workflow Name:</label>
                <input type="text" id="newWorkflowName" placeholder="e.g., Startup Sequence">
            </div>
            <div class="actions">
                <button onclick="confirmAddWorkflow()">Create</button>
                <button class="secondary" onclick="closeWorkflowModal()">Cancel</button>
            </div>
        </div>
    </div>

    <!-- Delete Confirmation Modal -->
    <div id="deleteConfirmModal" class="modal">
        <div class="modal-content">
            <h2>Confirm Delete</h2>
            <p id="deleteConfirmMessage">Are you sure you want to delete this?</p>
            <div class="actions">
                <button onclick="confirmDeleteAction()">Delete</button>
                <button class="secondary" onclick="closeDeleteModal()">Cancel</button>
            </div>
        </div>
    </div>

    <!-- Add Step Modal -->
    <div id="addStepModal" class="modal">
        <div class="modal-content">
            <h2>Add Workflow Step</h2>
            <div class="form-group">
                <label>Step Type:</label>
                <select id="stepType" onchange="updateStepForm()">
                    <option value="source">Source Workspace</option>
                    <option value="launch">Launch File</option>
                    <option value="run">Run Node</option>
                    <option value="command">Run Command</option>
                    <option value="delay">Delay</option>
                </select>
            </div>
            <div id="stepFormContainer"></div>
            
            <div class="form-group" style="margin-top: 15px; border-top: 1px solid var(--vscode-input-border); padding-top: 10px;">
                <label style="display: flex; align-items: center; gap: 8px; cursor: pointer;">
                    <input type="checkbox" id="stepSameTerminal">
                    <span>Run in previous terminal</span>
                </label>
                <small style="color: var(--vscode-descriptionForeground); display: block; margin-top: 4px;">
                    If checked, this command will run in the same terminal as the previous step.
                </small>
            </div>

            <div class="actions">
                <button onclick="confirmAddStep()">Add Step</button>
                <button class="secondary" onclick="closeModal()">Cancel</button>
            </div>
        </div>
    </div>

    <script>
        const vscode = acquireVsCodeApi();
        let config = { workflows: [], activeWorkflow: null };
        let availableNodes = [];
        let availableLaunchFiles = [];
        let currentWorkflowIndex = -1;
        let deleteAction = null;

        // Request initial config
        vscode.postMessage({ command: 'getConfig' });

        // Listen for messages from extension
        window.addEventListener('message', event => {
            const message = event.data;
            if (message.command === 'loadConfig') {
                config = message.config;
                if (!config.workflows) {
                    config.workflows = [];
                }
                renderWorkflows();
            } else if (message.command === 'loadItems') {
                availableNodes = message.nodes || [];
                availableLaunchFiles = message.launchFiles || [];
            }
        });

        function renderWorkflows() {
            const container = document.getElementById('workflowList');
            if (config.workflows.length === 0) {
                container.innerHTML = '<p style="text-align: center; color: var(--vscode-descriptionForeground);">No workflows configured. Click "New Workflow" to get started.</p>';
                return;
            }

            container.innerHTML = config.workflows.map((workflow, index) => \`
                <div class="workflow-item">
                    <div style="display: flex; justify-content: space-between; align-items: center;">
                        <h3>\${workflow.name}</h3>
                        <div>
                            <button class="secondary" onclick="deleteWorkflow(\${index})">${icons.trash} Delete</button>
                        </div>
                    </div>
                    <div class="step-list">
                        \${workflow.steps.map((step, stepIndex) => \`
                            <div class="step-item \${!step.enabled ? 'disabled' : ''}">
                                <input type="checkbox" \${step.enabled ? 'checked' : ''} onchange="toggleStep(\${index}, \${stepIndex})">
                                <span class="step-type">
                                    \${step.type.toUpperCase()}
                                    \${step.sameTerminal ? '<span title="Runs in previous terminal">${icons.link}</span>' : ''}
                                </span>
                                <span class="step-description">\${getStepDescription(step)}</span>
                                <div class="step-controls">
                                    <button class="secondary" onclick="moveStep(\${index}, \${stepIndex}, -1)" \${stepIndex === 0 ? 'disabled' : ''}>${icons.up}</button>
                                    <button class="secondary" onclick="moveStep(\${index}, \${stepIndex}, 1)" \${stepIndex === workflow.steps.length - 1 ? 'disabled' : ''}>${icons.down}</button>
                                    <button class="secondary" onclick="deleteStep(\${index}, \${stepIndex})">${icons.trash}</button>
                                </div>
                            </div>
                        \`).join('')}
                    </div>
                    <button onclick="openAddStepModal(\${index})">+ Add Step</button>
                </div>
            \`).join('');
        }

        function getStepDescription(step) {
            switch (step.type) {
                case 'source':
                    return 'Source install/setup.bash';
                case 'launch':
                    return \`\${step.config.package}/\${step.config.launchFile}\`;
                case 'run':
                    return \`\${step.config.package}/\${step.config.nodeName}\`;
                case 'command':
                    return step.config.command || 'No command entered';
                case 'delay':
                    return \`Wait \${step.config.delayMs || 1000}ms\`;
                default:
                    return step.config.description || 'No description';
            }
        }

        function addWorkflow() {
            document.getElementById('newWorkflowName').value = '';
            document.getElementById('addWorkflowModal').classList.add('show');
            document.getElementById('newWorkflowName').focus();
        }

        function closeWorkflowModal() {
            document.getElementById('addWorkflowModal').classList.remove('show');
        }

        function confirmAddWorkflow() {
            const name = document.getElementById('newWorkflowName').value;
            if (!name) {return;}
            
            config.workflows.push({
                name: name,
                steps: []
            });
            
            closeWorkflowModal();
            renderWorkflows();
        }

        function deleteWorkflow(index) {
            document.getElementById('deleteConfirmMessage').innerText = \`Delete workflow "\${config.workflows[index].name}"?\`;
            deleteAction = () => {
                config.workflows.splice(index, 1);
                renderWorkflows();
            };
            document.getElementById('deleteConfirmModal').classList.add('show');
        }

        function closeDeleteModal() {
            document.getElementById('deleteConfirmModal').classList.remove('show');
            deleteAction = null;
        }

        function confirmDeleteAction() {
            if (deleteAction) {
                deleteAction();
            }
            closeDeleteModal();
        }

        function openAddStepModal(workflowIndex) {
            currentWorkflowIndex = workflowIndex;
            document.getElementById('addStepModal').classList.add('show');
            // Reset checkbox
            document.getElementById('stepSameTerminal').checked = false;
            updateStepForm();
        }

        function closeModal() {
            document.getElementById('addStepModal').classList.remove('show');
        }

        function updateStepForm() {
            const type = document.getElementById('stepType').value;
            const container = document.getElementById('stepFormContainer');
            
            let formHtml = '';
            
            if (type === 'launch') {
                const packages = [...new Set(availableLaunchFiles.map(f => f.package))].sort();
                const pkgOptions = packages.map(p => \`<option value="\${p}">\${p}</option>\`).join('');
                
                formHtml = \`
                    <div class="form-group">
                        <label>Package Name:</label>
                        <select id="stepPackage" onchange="updateLaunchFiles()">
                            <option value="">Select Package</option>
                            \${pkgOptions}
                        </select>
                    </div>
                    <div class="form-group">
                        <label>Launch File:</label>
                        <select id="stepLaunchFile">
                            <option value="">Select Package First</option>
                        </select>
                    </div>
                \`;
            } else if (type === 'run') {
                const packages = [...new Set(availableNodes.map(n => n.package))].sort();
                const pkgOptions = packages.map(p => \`<option value="\${p}">\${p}</option>\`).join('');
                
                formHtml = \`
                    <div class="form-group">
                        <label>Package Name:</label>
                        <select id="stepPackage" onchange="updateNodes()">
                            <option value="">Select Package</option>
                            \${pkgOptions}
                        </select>
                    </div>
                    <div class="form-group">
                        <label>Node Name:</label>
                        <select id="stepNodeName">
                            <option value="">Select Package First</option>
                        </select>
                    </div>
                \`;
            } else if (type === 'source') {
                formHtml = '<p>This step will source the workspace setup.bash file.</p>';
            } else if (type === 'command') {
                formHtml = \`
                    <div class="form-group">
                        <label>Command:</label>
                        <input type="text" id="stepCommand" placeholder="e.g., echo 'Hello World'">
                    </div>
                \`;
            } else if (type === 'delay') {
                formHtml = \`
                    <div class="form-group">
                        <label>Delay (milliseconds):</label>
                        <input type="number" id="stepDelayMs" value="1000" min="0">
                    </div>
                \`;
            }
            
            container.innerHTML = formHtml;
        }

        function updateLaunchFiles() {
            const pkg = document.getElementById('stepPackage').value;
            const fileSelect = document.getElementById('stepLaunchFile');
            
            if (!pkg) {
                fileSelect.innerHTML = '<option value="">Select Package First</option>';
                return;
            }
            
            const files = availableLaunchFiles.filter(f => f.package === pkg).map(f => f.name).sort();
            fileSelect.innerHTML = files.map(f => \`<option value="\${f}">\${f}</option>\`).join('');
        }

        function updateNodes() {
            const pkg = document.getElementById('stepPackage').value;
            const nodeSelect = document.getElementById('stepNodeName');
            
            if (!pkg) {
                nodeSelect.innerHTML = '<option value="">Select Package First</option>';
                return;
            }
            
            const nodes = availableNodes.filter(n => n.package === pkg).map(n => n.name).sort();
            nodeSelect.innerHTML = nodes.map(n => \`<option value="\${n}">\${n}</option>\`).join('');
        }

        function confirmAddStep() {
            const type = document.getElementById('stepType').value;
            const sameTerminal = document.getElementById('stepSameTerminal').checked;
            
            const step = {
                id: Date.now().toString() + Math.random().toString(36).substr(2, 9),
                type: type,
                enabled: true,
                sameTerminal: sameTerminal,
                config: {}
            };
            
            switch (type) {
                case 'launch':
                    step.config.package = document.getElementById('stepPackage').value;
                    step.config.launchFile = document.getElementById('stepLaunchFile').value;
                    break;
                case 'run':
                    step.config.package = document.getElementById('stepPackage').value;
                    step.config.nodeName = document.getElementById('stepNodeName').value;
                    break;
                case 'command':
                    step.config.command = document.getElementById('stepCommand').value;
                    break;
                case 'delay':
                    step.config.delayMs = parseInt(document.getElementById('stepDelayMs').value);
                    break;
            }
            
            config.workflows[currentWorkflowIndex].steps.push(step);
            closeModal();
            renderWorkflows();
        }

        function toggleStep(workflowIndex, stepIndex) {
            config.workflows[workflowIndex].steps[stepIndex].enabled = 
                !config.workflows[workflowIndex].steps[stepIndex].enabled;
            renderWorkflows();
        }

        function moveStep(workflowIndex, stepIndex, direction) {
            const steps = config.workflows[workflowIndex].steps;
            const newIndex = stepIndex + direction;
            
            if (newIndex < 0 || newIndex >= steps.length) {return;}
            
            [steps[stepIndex], steps[newIndex]] = [steps[newIndex], steps[stepIndex]];
            renderWorkflows();
        }

        function deleteStep(workflowIndex, stepIndex) {
            config.workflows[workflowIndex].steps.splice(stepIndex, 1);
            renderWorkflows();
        }

        function saveConfig() {
            vscode.postMessage({
                command: 'save',
                config: config
            });
        }
    </script>
</body>
</html>`;
    }
}
