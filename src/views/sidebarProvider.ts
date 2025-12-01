import * as vscode from 'vscode';
import { LaunchService, LaunchFile } from '../services/launchService';
import { NodeDiscoveryService, DiscoveredNode } from '../services/nodeDiscoveryService';
import { getWorkspaceRoot } from '../utils/workspaceUtils';

export class ROS2SidebarProvider implements vscode.TreeDataProvider<ROS2TreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<ROS2TreeItem | undefined | null | void> = new vscode.EventEmitter<ROS2TreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<ROS2TreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    constructor(
        private launchService: LaunchService,
        private nodeDiscoveryService: NodeDiscoveryService
    ) { }

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: ROS2TreeItem): vscode.TreeItem {
        return element;
    }

    async getChildren(element?: ROS2TreeItem): Promise<ROS2TreeItem[]> {
        if (!element) {
            // Root level
            return [
                new ROS2TreeItem('Workspace', undefined, vscode.TreeItemCollapsibleState.Expanded, new vscode.ThemeIcon('folder'), 'workspace'),
                new ROS2TreeItem('Launch Files', undefined, vscode.TreeItemCollapsibleState.Collapsed, new vscode.ThemeIcon('rocket'), 'launch-files'),
                new ROS2TreeItem('Nodes', undefined, vscode.TreeItemCollapsibleState.Collapsed, new vscode.ThemeIcon('symbol-event'), 'nodes')
            ];
        }

        if (element.contextValue === 'workspace') {
            return [
                new ROS2TreeItem('Build', 'ros2.build', vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('tools')),
                new ROS2TreeItem('Clean Build', 'ros2.cleanBuild', vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('trash')),
                new ROS2TreeItem('Create Package', 'ros2.createPackage', vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('package')),
                new ROS2TreeItem('Create Node', 'ros2.createNode', vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('file-code')),
                new ROS2TreeItem('Generate .gitignore', 'ros2.generateGitignore', vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('git-merge'))
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

    private async getLaunchFiles(): Promise<ROS2TreeItem[]> {
        const workspaceRoot = getWorkspaceRoot();
        if (!workspaceRoot) {
            return [];
        }

        const launchFiles = await this.launchService.findLaunchFiles(workspaceRoot);

        // Group by package
        const groupedFiles = new Map<string, LaunchFile[]>();
        for (const file of launchFiles) {
            if (!groupedFiles.has(file.package)) {
                groupedFiles.set(file.package, []);
            }
            groupedFiles.get(file.package)?.push(file);
        }

        const items: ROS2TreeItem[] = [];
        for (const [pkg, files] of groupedFiles) {
            for (const file of files) {
                const icon = file.type === 'python' ? new vscode.ThemeIcon('file-code') : new vscode.ThemeIcon('file-xml');
                const item = new ROS2TreeItem(file.name, 'ros2.runLaunchFile', vscode.TreeItemCollapsibleState.None, icon);
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
        return items.sort((a, b) => (typeof a.label === 'string' ? a.label : a.label.label).localeCompare(typeof b.label === 'string' ? b.label : b.label.label));
    }

    private async getNodes(): Promise<ROS2TreeItem[]> {
        const workspaceRoot = getWorkspaceRoot();
        if (!workspaceRoot) {
            return [];
        }

        const nodes = await this.nodeDiscoveryService.findNodes(workspaceRoot);

        // Group by package
        const groupedNodes = new Map<string, DiscoveredNode[]>();
        for (const node of nodes) {
            if (!groupedNodes.has(node.package)) {
                groupedNodes.set(node.package, []);
            }
            groupedNodes.get(node.package)?.push(node);
        }

        const items: ROS2TreeItem[] = [];
        for (const [pkg, pkgNodes] of groupedNodes) {
            for (const node of pkgNodes) {
                const item = new ROS2TreeItem(node.name, 'ros2.createRunConfiguration', vscode.TreeItemCollapsibleState.None, new vscode.ThemeIcon('symbol-event'));
                item.description = node.package;
                item.tooltip = `Package: ${node.package}\nType: ${node.type}`;
                item.command = {
                    command: 'ros2.createRunConfiguration',
                    title: 'Create Run Configuration',
                    arguments: [node]
                };
                items.push(item);
            }
        }
        return items.sort((a, b) => (typeof a.label === 'string' ? a.label : a.label.label).localeCompare(typeof b.label === 'string' ? b.label : b.label.label));
    }
}

class ROS2TreeItem extends vscode.TreeItem {
    constructor(
        public readonly label: string | vscode.TreeItemLabel,
        public readonly commandId: string | undefined,
        public readonly collapsibleState: vscode.TreeItemCollapsibleState,
        public readonly icon?: vscode.ThemeIcon,
        public readonly contextValue?: string
    ) {
        super(label, collapsibleState);

        if (commandId) {
            this.command = {
                command: commandId,
                title: typeof label === 'string' ? label : label.label,
                arguments: []
            };
        }

        if (icon) {
            this.iconPath = icon;
        }

        this.contextValue = contextValue;
    }
}
