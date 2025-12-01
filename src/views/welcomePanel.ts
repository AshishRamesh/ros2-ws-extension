import * as vscode from 'vscode';
import { EnvironmentService } from '../services/environmentService';

export class WelcomePanel {
    public static currentPanel: WelcomePanel | undefined;
    private readonly _panel: vscode.WebviewPanel;
    private _disposables: vscode.Disposable[] = [];
    private readonly _environmentService: EnvironmentService;

    private constructor(panel: vscode.WebviewPanel, extensionUri: vscode.Uri, environmentService: EnvironmentService) {
        this._panel = panel;
        this._environmentService = environmentService;

        // Set the webview's initial html content
        this._panel.webview.html = this._getHtmlForWebview(this._panel.webview);

        // Listen for when the panel is disposed
        this._panel.onDidDispose(() => this.dispose(), null, this._disposables);

        // Handle messages from the webview
        this._panel.webview.onDidReceiveMessage(
            message => {
                switch (message.command) {
                    case 'initWorkspace':
                        vscode.commands.executeCommand('ros2.initWorkspace');
                        return;
                    case 'build':
                        vscode.commands.executeCommand('ros2.build');
                        return;
                    case 'cleanBuild':
                        vscode.commands.executeCommand('ros2.cleanBuild');
                        return;
                    case 'runLaunch':
                        vscode.commands.executeCommand('ros2.runLaunchFile');
                        return;
                }
            },
            null,
            this._disposables
        );
    }

    public static createOrShow(extensionUri: vscode.Uri, environmentService: EnvironmentService) {
        // If we already have a panel, show it
        if (WelcomePanel.currentPanel) {
            WelcomePanel.currentPanel._panel.reveal(vscode.ViewColumn.One);
            return;
        }

        // Otherwise, create a new panel
        const panel = vscode.window.createWebviewPanel(
            'ros2Welcome',
            'ROS 2 Developer Tools',
            vscode.ViewColumn.One,
            {
                enableScripts: true,
                retainContextWhenHidden: true
            }
        );

        WelcomePanel.currentPanel = new WelcomePanel(panel, extensionUri, environmentService);
    }

    public dispose() {
        WelcomePanel.currentPanel = undefined;

        // Clean up our resources
        this._panel.dispose();

        while (this._disposables.length) {
            const disposable = this._disposables.pop();
            if (disposable) {
                disposable.dispose();
            }
        }
    }

    private _getHtmlForWebview(webview: vscode.Webview): string {
        const envInfo = this._environmentService.detectEnvironment();
        const distroDisplay = envInfo.rosDistro ? `ROS 2 ${envInfo.rosDistro.charAt(0).toUpperCase() + envInfo.rosDistro.slice(1)}` : 'ROS 2 Not Detected';
        const isWorkspace = envInfo.isWorkspace;
        const disabledAttr = isWorkspace ? '' : 'disabled';
        const disabledClass = isWorkspace ? '' : 'disabled';
        const warningDisplay = isWorkspace ? 'none' : 'block';

        return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROS 2 Developer Tools</title>
    <style>
        body {
            font-family: var(--vscode-font-family);
            color: var(--vscode-foreground);
            background-color: var(--vscode-editor-background);
            padding: 20px;
            margin: 0;
        }
        
        h1 {
            color: var(--vscode-foreground);
            font-size: 28px;
            margin-bottom: 10px;
        }
        
        .subtitle {
            color: var(--vscode-descriptionForeground);
            margin-bottom: 30px;
            font-size: 14px;
        }

        .banner {
            background-color: var(--vscode-activityBar-background);
            color: var(--vscode-activityBar-foreground);
            padding: 10px 15px;
            border-radius: 4px;
            margin-bottom: 20px;
            display: inline-block;
            font-weight: bold;
        }

        .warning-banner {
            background-color: var(--vscode-inputValidation-warningBackground);
            border: 1px solid var(--vscode-inputValidation-warningBorder);
            padding: 10px;
            margin-bottom: 20px;
            border-radius: 4px;
            display: ${warningDisplay};
        }
        
        .button-container {
            display: flex;
            flex-direction: column;
            gap: 15px;
            max-width: 400px;
        }
        
        button {
            background-color: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            padding: 12px 24px;
            font-size: 14px;
            cursor: pointer;
            border-radius: 4px;
            transition: background-color 0.2s;
            text-align: left;
        }
        
        button:hover {
            background-color: var(--vscode-button-hoverBackground);
        }
        
        button:active {
            opacity: 0.8;
        }

        button.disabled {
            opacity: 0.5;
            cursor: not-allowed;
            background-color: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
        }
        
        .button-icon {
            margin-right: 8px;
        }
        
        .section {
            margin-top: 40px;
        }
        
        .section-title {
            font-size: 18px;
            margin-bottom: 15px;
            color: var(--vscode-foreground);
        }
    </style>
</head>
<body>
    <h1>🤖 ROS 2 Developer Tools</h1>
    <div class="subtitle">Streamline your ROS 2 development workflow</div>

    <div class="banner">
        ${distroDisplay}
    </div>

    <div class="warning-banner">
        ⚠️ Not a ROS 2 workspace. Please initialize a workspace or open a folder with a src/ directory.
    </div>
    
    <div class="section">
        <div class="section-title">Workspace Management</div>
        <div class="button-container">
            <button onclick="sendMessage('initWorkspace')">
                <span class="button-icon">📁</span>Initialize Workspace
            </button>
            <button onclick="sendMessage('build')" ${disabledAttr} class="${disabledClass}">
                <span class="button-icon">🔨</span>Build Workspace
            </button>
            <button onclick="sendMessage('cleanBuild')" ${disabledAttr} class="${disabledClass}">
                <span class="button-icon">🧹</span>Clean Build
            </button>
        </div>
    </div>

    <div class="section">
        <div class="section-title">Launch & Simulation</div>
        <div class="button-container">
            <button onclick="sendMessage('runLaunch')" ${disabledAttr} class="${disabledClass}">
                <span class="button-icon">🚀</span>Run Launch File
            </button>
        </div>
    </div>
    
    <script>
        const vscode = acquireVsCodeApi();
        
        function sendMessage(command) {
            vscode.postMessage({ command: command });
        }
    </script>
</body>
</html>`;
    }
}
