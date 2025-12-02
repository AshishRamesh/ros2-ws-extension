import * as vscode from 'vscode';
import { TopicsService } from '../services/topicsService';
import { EchoService } from '../services/echoService';

export class TopicsTreeViewProvider implements vscode.TreeDataProvider<TopicsTreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<TopicsTreeItem | undefined | null | void> = new vscode.EventEmitter<TopicsTreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<TopicsTreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    private activeEchoTopic: string | null = null;

    constructor(
        private topicsService: TopicsService,
        private echoService: EchoService
    ) { }

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: TopicsTreeItem): vscode.TreeItem {
        return element;
    }

    async getChildren(element?: TopicsTreeItem): Promise<TopicsTreeItem[]> {
        if (!element) {
            return this.getTopicsChildren();
        }

        if (element.contextValue === 'topic-item') {
            return this.getTopicEchoChildren(element.label as string);
        }

        return [];
    }

    private async getTopicsChildren(): Promise<TopicsTreeItem[]> {
        const topics = await this.topicsService.listTopics();

        const items: TopicsTreeItem[] = [];

        for (const topic of topics) {
            const item = new TopicsTreeItem(
                topic,
                undefined,
                vscode.TreeItemCollapsibleState.Collapsed,
                new vscode.ThemeIcon('radio-tower'),
                'topic-item'
            );
            item.tooltip = `Click to expand and view metadata`;
            items.push(item);
        }
        return items;
    }

    private async getTopicEchoChildren(topic: string): Promise<TopicsTreeItem[]> {
        const info = await this.topicsService.getTopicInfo(topic);

        if (this.activeEchoTopic !== topic) {
            this.activeEchoTopic = topic;
            this.echoService.stopEcho();

            this.echoService.startEcho(topic, () => {
                this.refresh();
            });
        }

        const items: TopicsTreeItem[] = [
            new TopicsTreeItem(`Type: ${info.type}`, undefined, vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('symbol-class'), 'topic-info'),
            new TopicsTreeItem(`Publishers: ${info.publishers}`, undefined, vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('account'), 'topic-info'),
            new TopicsTreeItem(`Subscribers: ${info.subscribers}`, undefined, vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('account'), 'topic-info'),
            new TopicsTreeItem(`Frequency: ${this.echoService.getFrequency().toFixed(2)} Hz`, undefined, vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('pulse'), 'topic-info'),
            new TopicsTreeItem(`Messages: ${this.echoService.getMessageCount()}`, undefined, vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('list-ordered'), 'topic-info')
        ];

        const viewMessagesItem = new TopicsTreeItem(
            'Click here to view messages',
            'ros2.viewTopicMessages',
            vscode.TreeItemCollapsibleState.None,
            new vscode.ThemeIcon('output'),
            'view-messages'
        );
        viewMessagesItem.command = {
            command: 'ros2.viewTopicMessages',
            title: 'View Messages',
            arguments: [topic]
        };
        items.push(viewMessagesItem);

        return items;
    }
}

class TopicsTreeItem extends vscode.TreeItem {
    constructor(
        public readonly label: string,
        public readonly commandId: string | undefined,
        public readonly collapsibleState: vscode.TreeItemCollapsibleState,
        public readonly icon?: vscode.ThemeIcon,
        public readonly contextValue?: string
    ) {
        super(label, collapsibleState);

        if (commandId) {
            this.command = {
                command: commandId,
                title: label,
                arguments: []
            };
        }

        if (icon) {
            this.iconPath = icon;
        }

        this.contextValue = contextValue;
    }
}
