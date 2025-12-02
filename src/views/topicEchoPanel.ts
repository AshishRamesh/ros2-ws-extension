import * as vscode from 'vscode';
import { EchoService } from '../services/echoService';

export class TopicEchoPanel {
    public static currentPanel: TopicEchoPanel | undefined;
    private readonly _panel: vscode.WebviewPanel;
    private _disposables: vscode.Disposable[] = [];
    private currentTopic: string;

    public static createOrShow(extensionUri: vscode.Uri, topic: string, echoService: EchoService) {
        const column = vscode.window.activeTextEditor
            ? vscode.window.activeTextEditor.viewColumn
            : undefined;

        // If we already have a panel, show it
        if (TopicEchoPanel.currentPanel) {
            TopicEchoPanel.currentPanel.updateTopic(topic, echoService);
            TopicEchoPanel.currentPanel._panel.reveal(column);
            return;
        }

        // Otherwise, create a new panel
        const panel = vscode.window.createWebviewPanel(
            'ros2TopicEcho',
            `ROS 2 Echo: ${topic}`,
            column || vscode.ViewColumn.One,
            {
                enableScripts: true,
                localResourceRoots: [extensionUri]
            }
        );

        TopicEchoPanel.currentPanel = new TopicEchoPanel(panel, extensionUri, topic, echoService);
    }

    private constructor(panel: vscode.WebviewPanel, extensionUri: vscode.Uri, topic: string, echoService: EchoService) {
        this._panel = panel;
        this.currentTopic = topic;

        // Set the webview's initial html content
        this._update(topic);

        // Start echo
        this.startEcho(topic, echoService);

        // Listen for when the panel is disposed
        this._panel.onDidDispose(() => this.dispose(), null, this._disposables);

        // Handle messages from the webview
        this._panel.webview.onDidReceiveMessage(
            message => {
                switch (message.command) {
                    case 'clear':
                        this._panel.webview.postMessage({ type: 'clear' });
                        return;
                }
            },
            null,
            this._disposables
        );
    }

    public updateTopic(topic: string, echoService: EchoService) {
        this.currentTopic = topic;
        this._panel.title = `ROS 2 Echo: ${topic}`;
        this._update(topic);
        this.startEcho(topic, echoService);
    }

    private startEcho(topic: string, echoService: EchoService) {
        echoService.stopEcho();

        echoService.startEcho(topic, (message) => {
            this._panel.webview.postMessage({
                type: 'message',
                data: message.data,
                timestamp: message.timestamp.toISOString(),
                count: echoService.getMessageCount(),
                frequency: echoService.getFrequency().toFixed(2)
            });
        });
    }

    public dispose() {
        TopicEchoPanel.currentPanel = undefined;

        this._panel.dispose();

        while (this._disposables.length) {
            const x = this._disposables.pop();
            if (x) {
                x.dispose();
            }
        }
    }

    private _update(topic: string) {
        const webview = this._panel.webview;
        this._panel.webview.html = this._getHtmlForWebview(webview, topic);
    }

    private _getHtmlForWebview(webview: vscode.Webview, topic: string) {
        return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROS 2 Echo: ${topic}</title>
    <style>
        body {
            padding: 0;
            margin: 0;
            font-family: var(--vscode-font-family);
            color: var(--vscode-foreground);
            background: var(--vscode-editor-background);
        }
        .header {
            padding: 16px;
            background: var(--vscode-sideBar-background);
            border-bottom: 1px solid var(--vscode-panel-border);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .header h2 {
            margin: 0;
            font-size: 16px;
        }
        .stats {
            display: flex;
            gap: 16px;
            font-size: 12px;
            color: var(--vscode-descriptionForeground);
        }
        .controls {
            padding: 12px 16px;
            background: var(--vscode-sideBar-background);
            border-bottom: 1px solid var(--vscode-panel-border);
        }
        button {
            padding: 6px 12px;
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            cursor: pointer;
            font-size: 12px;
        }
        button:hover {
            background: var(--vscode-button-hoverBackground);
        }
        .messages {
            padding: 16px;
            height: calc(100vh - 120px);
            overflow-y: auto;
            font-family: var(--vscode-editor-font-family);
            font-size: 12px;
        }
        .message {
            margin-bottom: 16px;
            padding: 12px;
            background: var(--vscode-textCodeBlock-background);
            border-left: 3px solid var(--vscode-textLink-foreground);
            white-space: pre-wrap;
            word-wrap: break-word;
        }
        .message-header {
            color: var(--vscode-descriptionForeground);
            font-size: 11px;
            margin-bottom: 8px;
        }
        .separator {
            border-top: 1px dashed var(--vscode-panel-border);
            margin: 8px 0;
        }
    </style>
</head>
<body>
    <div class="header">
        <h2>Topic: ${topic}</h2>
        <div class="stats">
            <span>Frequency: <strong id="frequency">0.00 Hz</strong></span>
            <span>Messages: <strong id="count">0</strong></span>
        </div>
    </div>
    <div class="controls">
        <button onclick="clearMessages()">Clear Messages</button>
    </div>
    <div id="messages" class="messages"></div>

    <script>
        const vscode = acquireVsCodeApi();
        const messagesDiv = document.getElementById('messages');
        let totalMessageCount = 0;

        window.addEventListener('message', event => {
            const message = event.data;
            
            switch (message.type) {
                case 'message':
                    updateMessage(message.data, message.timestamp);
                    document.getElementById('frequency').textContent = message.frequency + ' Hz';
                    document.getElementById('count').textContent = message.count;
                    totalMessageCount = message.count;
                    break;
                case 'clear':
                    messagesDiv.innerHTML = '';
                    totalMessageCount = 0;
                    break;
            }
        });

        function updateMessage(data, timestamp) {
            const time = new Date(timestamp).toLocaleTimeString();
            messagesDiv.innerHTML = '<div class="message"><div class="message-header">#' + totalMessageCount + ' - ' + time + '</div><div class="separator"></div>' + escapeHtml(data) + '</div>';
        }

        function clearMessages() {
            vscode.postMessage({ command: 'clear' });
        }

        function escapeHtml(text) {
            const div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }
    </script>
</body>
</html>`;
    }
}
