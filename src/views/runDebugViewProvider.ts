import * as vscode from 'vscode';
import { LaunchService, LaunchFile } from '../services/launchService';
import { NodeDiscoveryService, DiscoveredNode } from '../services/nodeDiscoveryService';
import { getWorkspaceRoot } from '../utils/workspaceUtils';

export class RunDebugViewProvider implements vscode.TreeDataProvider<RunDebugTreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<RunDebugTreeItem | undefined | null | void> = new vscode.EventEmitter<RunDebugTreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<RunDebugTreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    constructor(
        private launchService: LaunchService,
        private nodeDiscoveryService: NodeDiscoveryService
    ) { }

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: RunDebugTreeItem): vscode.TreeItem {
        return element;
    }

    async getChildren(element?: RunDebugTreeItem): Promise<RunDebugTreeItem[]> {
        if (!element) {
            return [
                new RunDebugTreeItem('Launch Files', vscode.TreeItemCollapsibleState.Collapsed, new vscode.ThemeIcon('rocket'), 'launch-files'),
                new RunDebugTreeItem('Nodes', vscode.TreeItemCollapsibleState.Collapsed, new vscode.ThemeIcon('symbol-event'), 'nodes')
            ];
        }

        if (element.contextValue === 'launch-files') {
            return this.getLaunchFiles();
        }

        if (element.contextValue === 'nodes') {
            return this.getNodes();
        }

        return [];
    }

    private async getLaunchFiles(): Promise<RunDebugTreeItem[]> {
        const workspaceRoot = getWorkspaceRoot();
        if (!workspaceRoot) {
            return [];
        }

        const launchFiles = await this.launchService.findLaunchFiles(workspaceRoot);

        const groupedFiles = new Map<string, LaunchFile[]>();
        for (const file of launchFiles) {
            if (!groupedFiles.has(file.package)) {
                groupedFiles.set(file.package, []);
            }
            groupedFiles.get(file.package)?.push(file);
        }

        const items: RunDebugTreeItem[] = [];
        for (const [pkg, files] of groupedFiles) {
            for (const file of files) {
                const icon = file.type === 'python' ? new vscode.ThemeIcon('file-code') : new vscode.ThemeIcon('file-xml');
                const item = new RunDebugTreeItem(file.name, vscode.TreeItemCollapsibleState.None, icon);
                item.description = file.package;
                item.tooltip = file.path;
                item.command = {
                    command: 'ros2.runLaunchFile',
                    title: 'Run Launch File',
                    arguments: [file]
                };
                items.push(item);
            }
        }
        return items.sort((a, b) => (a.label as string).localeCompare(b.label as string));
    }

    private async getNodes(): Promise<RunDebugTreeItem[]> {
        const workspaceRoot = getWorkspaceRoot();
        if (!workspaceRoot) {
            return [];
        }

        const nodes = await this.nodeDiscoveryService.findNodes(workspaceRoot);

        const groupedNodes = new Map<string, DiscoveredNode[]>();
        for (const node of nodes) {
            if (!groupedNodes.has(node.package)) {
                groupedNodes.set(node.package, []);
            }
            groupedNodes.get(node.package)?.push(node);
        }

        const items: RunDebugTreeItem[] = [];
        for (const [pkg, pkgNodes] of groupedNodes) {
            for (const node of pkgNodes) {
                const item = new RunDebugTreeItem(node.name, vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('symbol-event'));
                item.description = node.package;
                item.tooltip = `Package: ${node.package}\nType: ${node.type}`;
                item.command = {
                    command: 'ros2.runNode',
                    title: 'Run Node',
                    arguments: [node]
                };
                items.push(item);
            }
        }
        return items.sort((a, b) => (a.label as string).localeCompare(b.label as string));
    }
}

class RunDebugTreeItem extends vscode.TreeItem {
    constructor(
        public readonly label: string,
        public readonly collapsibleState: vscode.TreeItemCollapsibleState,
        public readonly icon?: vscode.ThemeIcon,
        public readonly contextValue?: string
    ) {
        super(label, collapsibleState);
        if (icon) {
            this.iconPath = icon;
        }
        this.contextValue = contextValue;
    }
}
