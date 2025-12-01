import * as vscode from 'vscode';
import { NodeDiscoveryService, DiscoveredNode } from '../services/nodeDiscoveryService';
import { getWorkspaceRoot } from '../utils/workspaceUtils';

export class NodesSidebarProvider implements vscode.TreeDataProvider<NodeTreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<NodeTreeItem | undefined | null | void> = new vscode.EventEmitter<NodeTreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<NodeTreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    constructor(private nodeDiscoveryService: NodeDiscoveryService) { }

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: NodeTreeItem): vscode.TreeItem {
        return element;
    }

    async getChildren(element?: NodeTreeItem): Promise<NodeTreeItem[]> {
        if (element) {
            return [];
        }

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

        const items: NodeTreeItem[] = [];

        for (const [pkg, pkgNodes] of groupedNodes) {
            for (const node of pkgNodes) {
                items.push(new NodeTreeItem(node));
            }
        }

        return items.sort((a, b) => {
            const labelA = typeof a.label === 'string' ? a.label : a.label?.label || '';
            const labelB = typeof b.label === 'string' ? b.label : b.label?.label || '';
            return labelA.localeCompare(labelB);
        });
    }
}

class NodeTreeItem extends vscode.TreeItem {
    constructor(public readonly node: DiscoveredNode) {
        super(node.name, vscode.TreeItemCollapsibleState.None);

        this.description = node.package;
        this.tooltip = `Package: ${node.package}\nType: ${node.type}`;

        this.iconPath = new vscode.ThemeIcon('symbol-event');

        this.command = {
            command: 'ros2.createRunConfiguration',
            title: 'Create Run Configuration',
            arguments: [node] // We might need to handle this argument in the command handler if passed directly
        };
    }
}
