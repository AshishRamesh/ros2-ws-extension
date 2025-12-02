import * as vscode from 'vscode';

export class WorkspaceViewProvider implements vscode.TreeDataProvider<WorkspaceTreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<WorkspaceTreeItem | undefined | null | void> = new vscode.EventEmitter<WorkspaceTreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<WorkspaceTreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: WorkspaceTreeItem): vscode.TreeItem {
        return element;
    }

    getChildren(element?: WorkspaceTreeItem): Thenable<WorkspaceTreeItem[]> {
        if (element) {
            return Promise.resolve([]);
        }

        return Promise.resolve([
            new WorkspaceTreeItem('Build', 'ros2.build', new vscode.ThemeIcon('tools')),
            new WorkspaceTreeItem('Clean Build', 'ros2.cleanBuild', new vscode.ThemeIcon('trash')),
            new WorkspaceTreeItem('Create Package', 'ros2.createPackage', new vscode.ThemeIcon('package')),
            new WorkspaceTreeItem('Create Node', 'ros2.createNode', new vscode.ThemeIcon('file-code')),
            new WorkspaceTreeItem('Generate .gitignore', 'ros2.generateGitignore', new vscode.ThemeIcon('git-merge'))
        ]);
    }
}

class WorkspaceTreeItem extends vscode.TreeItem {
    constructor(
        public readonly label: string,
        public readonly commandId: string,
        public readonly icon: vscode.ThemeIcon
    ) {
        super(label, vscode.TreeItemCollapsibleState.None);
        this.iconPath = icon;
        this.command = {
            command: commandId,
            title: label,
            arguments: []
        };
    }
}
