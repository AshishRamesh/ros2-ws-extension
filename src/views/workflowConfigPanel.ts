import * as vscode from 'vscode';
import { Workflow, WorkflowStep, WorkflowConfiguration } from '../types/workflow';
import { LaunchService } from '../services/launchService';
import { NodeDiscoveryService } from '../services/nodeDiscoveryService';

export class WorkflowConfigPanel {
    public static currentPanel: WorkflowConfigPanel | undefined;
    private readonly _panel: vscode.WebviewPanel;
    private _disposables: vscode.Disposable[] = [];

    private constructor(
        panel: vscode.WebviewPanel,
        extensionUri: vscode.Uri,
        private onSave: (config: WorkflowConfiguration) => void,
        private launchService: LaunchService,
        private nodeDiscoveryService: NodeDiscoveryService
    ) {
        this._panel = panel;

        // Set the webview's initial html content
        this._update();

        // Listen for when the panel is disposed
        this._panel.onDidDispose(() => this.dispose(), null, this._disposables);

        // Handle messages from the webview
        this._panel.webview.onDidReceiveMessage(
            message => {
                switch (message.command) {
                    case 'save':
                        this.onSave(message.config);
                        vscode.window.showInformationMessage('Workflow configuration saved');
                        break;
                    case 'getConfig':
                        this.sendCurrentConfig();
                        this.sendAvailableItems();
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
        const column = vscode.ViewColumn.One;

        // If we already have a panel, show it
        if (WorkflowConfigPanel.currentPanel) {
            WorkflowConfigPanel.currentPanel._panel.reveal(column);
            return;
        }

        // Otherwise, create a new panel
        const panel = vscode.window.createWebviewPanel(
            'ros2WorkflowConfig',
            'ROS 2 Workflow Configuration',
            column,
            {
                enableScripts: true,
                retainContextWhenHidden: true,
                localResourceRoots: [extensionUri]
            }
        );

        WorkflowConfigPanel.currentPanel = new WorkflowConfigPanel(panel, extensionUri, onSave, launchService, nodeDiscoveryService);
    }

    public async sendAvailableItems() {
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
        if (!workspaceRoot) {
            return;
        }

        const nodes = await this.nodeDiscoveryService.findNodes(workspaceRoot);
        const launchFiles = await this.launchService.findLaunchFiles(workspaceRoot);

        this._panel.webview.postMessage({
            command: 'loadItems',
            nodes,
            launchFiles
        });
    }

    public sendCurrentConfig() {
        const config = vscode.workspace.getConfiguration('ros2Toolkit');
        const workflowConfig: WorkflowConfiguration = config.get('workflowSteps') || { workflows: [] };

        this._panel.webview.postMessage({
            command: 'loadConfig',
            config: workflowConfig
        });
    }

    public dispose() {
        WorkflowConfigPanel.currentPanel = undefined;

        this._panel.dispose();

        while (this._disposables.length) {
            const disposable = this._disposables.pop();
            if (disposable) {
                disposable.dispose();
            }
        }
    }

    private _update() {
        const webview = this._panel.webview;
        this._panel.webview.html = this._getHtmlForWebview(webview);
    }

    private _getHtmlForWebview(webview: vscode.Webview) {
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
    </style>
</head>
<body>
    <div class="container">
        <h1>🚀 ROS 2 Workflow Configuration</h1>
        
        <div class="actions">
            <button onclick="addWorkflow()">+ New Workflow</button>
            <button class="secondary" onclick="saveConfig()">💾 Save Configuration</button>
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
                    <option value="script">Run Script</option>
                    <option value="delay">Delay</option>
                </select>
            </div>
            <div id="stepFormContainer"></div>
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

        // ... [renderWorkflows, getStepDescription, addWorkflow, etc. remain same] ...

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
                            <button class="secondary" onclick="deleteWorkflow(\${index})">Delete</button>
                        </div>
                    </div>
                    <div class="step-list">
                        \${workflow.steps.map((step, stepIndex) => \`
                            <div class="step-item \${!step.enabled ? 'disabled' : ''}">
                                <input type="checkbox" \${step.enabled ? 'checked' : ''} onchange="toggleStep(\${index}, \${stepIndex})">
                                <span class="step-type">\${step.type.toUpperCase()}</span>
                                <span class="step-description">\${getStepDescription(step)}</span>
                                <div class="step-controls">
                                    <button class="secondary" onclick="moveStep(\${index}, \${stepIndex}, -1)" \${stepIndex === 0 ? 'disabled' : ''}>↑</button>
                                    <button class="secondary" onclick="moveStep(\${index}, \${stepIndex}, 1)" \${stepIndex === workflow.steps.length - 1 ? 'disabled' : ''}>↓</button>
                                    <button class="secondary" onclick="deleteStep(\${index}, \${stepIndex})">✕</button>
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
                case 'script':
                    return step.config.scriptPath || 'No script selected';
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
            } else if (type === 'script') {
                formHtml = \`
                    <div class="form-group">
                        <label>Script Path:</label>
                        <input type="text" id="stepScriptPath" placeholder="/path/to/script.sh">
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
            const step = {
                id: Date.now().toString() + Math.random().toString(36).substr(2, 9),
                type: type,
                enabled: true,
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
                case 'script':
                    step.config.scriptPath = document.getElementById('stepScriptPath').value;
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
