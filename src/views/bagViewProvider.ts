import * as vscode from 'vscode';

export class BagViewProvider implements vscode.TreeDataProvider<BagTreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<BagTreeItem | undefined | null | void> = new vscode.EventEmitter<BagTreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<BagTreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: BagTreeItem): vscode.TreeItem {
        return element;
    }

    getChildren(element?: BagTreeItem): Thenable<BagTreeItem[]> {
        if (element) {
            return Promise.resolve([]);
        }

        return Promise.resolve([
            new BagTreeItem('Record New Bag', 'ros2.recordBag', new vscode.ThemeIcon('record')),
            new BagTreeItem('Play Bag', 'ros2.playBag', new vscode.ThemeIcon('play'))
        ]);
    }
}

class BagTreeItem extends vscode.TreeItem {
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
