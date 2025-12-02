import * as vscode from 'vscode';
import { TopicsService } from '../services/topicsService';
import { EchoService, EchoMessage } from '../services/echoService';

export class TopicsViewProvider implements vscode.WebviewViewProvider {
    public static readonly viewType = 'ros2TopicsView';
    private _view?: vscode.WebviewView;
    private currentTopic: string | null = null;

    constructor(
        private readonly _extensionUri: vscode.Uri,
        private topicsService: TopicsService,
        private echoService: EchoService
    ) { }

    public resolveWebviewView(
        webviewView: vscode.WebviewView,
        context: vscode.WebviewViewResolveContext,
        _token: vscode.CancellationToken,
    ) {
        this._view = webviewView;

        webviewView.webview.options = {
            enableScripts: true,
            localResourceRoots: [this._extensionUri]
        };

        webviewView.webview.html = this._getHtmlForWebview(webviewView.webview);

        // Handle messages from the webview
        webviewView.webview.onDidReceiveMessage(async data => {
            switch (data.type) {
                case 'refresh':
                    await this.refreshTopics();
                    break;
                case 'selectTopic':
                    await this.selectTopic(data.topic);
                    break;
                case 'stopEcho':
                    this.stopEcho();
                    break;
            }
        });

        // Initial load
        this.refreshTopics();
    }

    private async refreshTopics() {
        const topics = await this.topicsService.listTopics();
        this._view?.webview.postMessage({ type: 'topicsList', topics });
    }

    private async selectTopic(topic: string) {
        this.currentTopic = topic;

        // Stop previous echo
        this.echoService.stopEcho();

        // Get topic info
        const info = await this.topicsService.getTopicInfo(topic);

        this._view?.webview.postMessage({
            type: 'topicInfo',
            topic,
            topicType: info.type,
            publishers: info.publishers,
            subscribers: info.subscribers
        });

        // Start new echo
        this.echoService.startEcho(topic, (message: EchoMessage) => {
            this._view?.webview.postMessage({
                type: 'echoMessage',
                data: message.data,
                timestamp: message.timestamp.toISOString(),
                count: this.echoService.getMessageCount(),
                frequency: this.echoService.getFrequency().toFixed(2)
            });
        });
    }

    private stopEcho() {
        this.echoService.stopEcho();
        this.currentTopic = null;
        this._view?.webview.postMessage({ type: 'echoStopped' });
    }

    private _getHtmlForWebview(webview: vscode.Webview) {
        return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ROS 2 Topics</title>
    <style>
        body {
            padding: 10px;
            font-family: var(--vscode-font-family);
            color: var(--vscode-foreground);
        }
        .controls {
            display: flex;
            gap: 8px;
            margin-bottom: 16px;
            align-items: center;
        }
        select {
            flex: 1;
            padding: 4px 8px;
            background: var(--vscode-input-background);
            color: var(--vscode-input-foreground);
            border: 1px solid var(--vscode-input-border);
        }
        button {
            padding: 4px 12px;
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            cursor: pointer;
        }
        button:hover {
            background: var(--vscode-button-hoverBackground);
        }
        .metadata {
            background: var(--vscode-editor-background);
            padding: 12px;
            margin-bottom: 12px;
            border-radius: 4px;
            font-size: 12px;
        }
        .metadata-row {
            display: flex;
            justify-content: space-between;
            margin-bottom: 4px;
        }
        .metadata-label {
            font-weight: bold;
            color: var(--vscode-descriptionForeground);
        }
        .echo-output {
            background: var(--vscode-editor-background);
            padding: 12px;
            height: 400px;
            overflow-y: auto;
            font-family: var(--vscode-editor-font-family);
            font-size: 12px;
            white-space: pre-wrap;
            border: 1px solid var(--vscode-panel-border);
        }
        .message-separator {
            border-top: 1px dashed var(--vscode-panel-border);
            margin: 8px 0;
        }
    </style>
</head>
<body>
    <div class="controls">
        <select id="topicSelect">
            <option value="">Select a Topic</option>
        </select>
        <button id="refreshBtn" title="Refresh Topics">🔄</button>
        <button id="stopBtn" title="Stop Echo">⏹</button>
    </div>

    <div id="metadata" class="metadata" style="display: none;">
        <div class="metadata-row">
            <span class="metadata-label">Topic:</span>
            <span id="topicName">-</span>
        </div>
        <div class="metadata-row">
            <span class="metadata-label">Type:</span>
            <span id="topicType">-</span>
        </div>
        <div class="metadata-row">
            <span class="metadata-label">Publishers:</span>
            <span id="publishers">-</span>
        </div>
        <div class="metadata-row">
            <span class="metadata-label">Subscribers:</span>
            <span id="subscribers">-</span>
        </div>
        <div class="metadata-row">
            <span class="metadata-label">Frequency:</span>
            <span id="frequency">-</span>
        </div>
        <div class="metadata-row">
            <span class="metadata-label">Messages:</span>
            <span id="messageCount">0</span>
        </div>
        <div class="metadata-row">
            <span class="metadata-label">Last Update:</span>
            <span id="lastTimestamp">-</span>
        </div>
    </div>

    <div id="echoOutput" class="echo-output"></div>

    <script>
        const vscode = acquireVsCodeApi();
        const topicSelect = document.getElementById('topicSelect');
        const refreshBtn = document.getElementById('refreshBtn');
        const stopBtn = document.getElementById('stopBtn');
        const metadata = document.getElementById('metadata');
        const echoOutput = document.getElementById('echoOutput');

        let messageBuffer = [];
        const MAX_MESSAGES = 50;

        topicSelect.addEventListener('change', (e) => {
            const topic = e.target.value;
            if (topic) {
                vscode.postMessage({ type: 'selectTopic', topic });
                echoOutput.textContent = 'Starting echo...\\n';
                messageBuffer = [];
            }
        });

        refreshBtn.addEventListener('click', () => {
            vscode.postMessage({ type: 'refresh' });
        });

        stopBtn.addEventListener('click', () => {
            vscode.postMessage({ type: 'stopEcho' });
        });

        window.addEventListener('message', event => {
            const message = event.data;
            
            switch (message.type) {
                case 'topicsList':
                    const currentValue = topicSelect.value;
                    topicSelect.innerHTML = '<option value="">Select a Topic</option>';
                    message.topics.forEach(topic => {
                        const option = document.createElement('option');
                        option.value = topic;
                        option.textContent = topic;
                        topicSelect.appendChild(option);
                    });
                    if (currentValue && message.topics.includes(currentValue)) {
                        topicSelect.value = currentValue;
                    }
                    break;

                case 'topicInfo':
                    metadata.style.display = 'block';
                    document.getElementById('topicName').textContent = message.topic;
                    document.getElementById('topicType').textContent = message.topicType;
                    document.getElementById('publishers').textContent = message.publishers;
                    document.getElementById('subscribers').textContent = message.subscribers;
                    break;

                case 'echoMessage':
                    messageBuffer.push(message.data);
                    if (messageBuffer.length > MAX_MESSAGES) {
                        messageBuffer.shift();
                    }
                    echoOutput.textContent = messageBuffer.join('\\n---\\n');
                    echoOutput.scrollTop = echoOutput.scrollHeight;
                    
                    document.getElementById('frequency').textContent = message.frequency + ' Hz';
                    document.getElementById('messageCount').textContent = message.count;
                    document.getElementById('lastTimestamp').textContent = new Date(message.timestamp).toLocaleTimeString();
                    break;

                case 'echoStopped':
                    echoOutput.textContent += '\\n\\n[Echo stopped]';
                    metadata.style.display = 'none';
                    break;
            }
        });
    </script>
</body>
</html>`;
    }
}
