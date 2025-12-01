import * as vscode from 'vscode';

export class ROS2SidebarProvider implements vscode.TreeDataProvider<ROS2TreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<ROS2TreeItem | undefined | null | void> = new vscode.EventEmitter<ROS2TreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<ROS2TreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    // Build options state
    private symlinkInstall: boolean = true;
    private addGitignore: boolean = true;

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: ROS2TreeItem): vscode.TreeItem {
        return element;
    }

    getChildren(element?: ROS2TreeItem): Thenable<ROS2TreeItem[]> {
        if (!element) {
            // Root level - show workspace section
            return Promise.resolve([
                new ROS2TreeItem(
                    'Workspace',
                    undefined,
                    vscode.TreeItemCollapsibleState.Expanded,
                    '📁',
                    'workspace'
                )
            ]);
        } else if (element.contextValue === 'workspace') {
            // Workspace children
            return Promise.resolve([
                new ROS2TreeItem(
                    'Build',
                    'ros2.build',
                    vscode.TreeItemCollapsibleState.None,
                    '🔨',
                    'build'
                ),
                new ROS2TreeItem(
                    'Clean Build',
                    'ros2.cleanBuild',
                    vscode.TreeItemCollapsibleState.None,
                    '🧹',
                    'clean-build'
                ),
                new ROS2TreeItem(
                    'Create Package',
                    'ros2.createPackage',
                    vscode.TreeItemCollapsibleState.None,
                    '📦',
                    'create-package'
                ),
                new ROS2TreeItem(
                    'Create Node',
                    'ros2.createNode',
                    vscode.TreeItemCollapsibleState.None,
                    '📝',
                    'create-node'
                ),
                new ROS2TreeItem(
                    'Generate .gitignore',
                    'ros2.generateGitignore',
                    vscode.TreeItemCollapsibleState.None,
                    '👁',
                    'generate-gitignore'
                )
            ]);
        }

        return Promise.resolve([]);
    }
}

class ROS2TreeItem extends vscode.TreeItem {
    constructor(
        public readonly label: string,
        public readonly commandId: string | undefined,
        public readonly collapsibleState: vscode.TreeItemCollapsibleState,
        public readonly icon?: string,
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
            this.description = icon;
        }

        this.contextValue = contextValue;
    }
}
