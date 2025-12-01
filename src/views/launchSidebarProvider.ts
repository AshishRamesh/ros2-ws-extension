import * as vscode from 'vscode';
import { LaunchService, LaunchFile } from '../services/launchService';
import { getWorkspaceRoot } from '../utils/workspaceUtils';

export class LaunchSidebarProvider implements vscode.TreeDataProvider<LaunchTreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<LaunchTreeItem | undefined | null | void> = new vscode.EventEmitter<LaunchTreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<LaunchTreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    constructor(private launchService: LaunchService) { }

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: LaunchTreeItem): vscode.TreeItem {
        return element;
    }

    async getChildren(element?: LaunchTreeItem): Promise<LaunchTreeItem[]> {
        if (element) {
            return [];
        }

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

        const items: LaunchTreeItem[] = [];

        // Create tree items
        for (const [pkg, files] of groupedFiles) {
            // Add package separator/header if needed, but for now flat list with description
            for (const file of files) {
                items.push(new LaunchTreeItem(file));
            }
        }

        return items.sort((a, b) => {
            const labelA = typeof a.label === 'string' ? a.label : a.label?.label || '';
            const labelB = typeof b.label === 'string' ? b.label : b.label?.label || '';
            return labelA.localeCompare(labelB);
        });
    }
}

class LaunchTreeItem extends vscode.TreeItem {
    constructor(public readonly launchFile: LaunchFile) {
        super(launchFile.name, vscode.TreeItemCollapsibleState.None);

        this.description = launchFile.package;
        this.tooltip = launchFile.path;

        // Set icon based on type
        if (launchFile.type === 'python') {
            this.iconPath = new vscode.ThemeIcon('file-code');
        } else {
            this.iconPath = new vscode.ThemeIcon('file-xml');
        }

        this.command = {
            command: 'ros2.runLaunchFile',
            title: 'Run Launch File',
            arguments: [launchFile]
        };
    }
}
