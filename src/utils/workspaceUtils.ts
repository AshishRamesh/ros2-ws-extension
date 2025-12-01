import * as vscode from 'vscode';

/**
 * Finds the ROS 2 workspace root
 */
export function getWorkspaceRoot(): string | undefined {
    const workspaceFolders = vscode.workspace.workspaceFolders;
    if (!workspaceFolders || workspaceFolders.length === 0) {
        return undefined;
    }
    return workspaceFolders[0].uri.fsPath;
}

/**
 * Validates if the given path is a valid ROS 2 workspace
 */
export function isValidROS2Workspace(workspacePath: string): boolean {
    const fs = require('fs');
    const path = require('path');

    const srcPath = path.join(workspacePath, 'src');
    return fs.existsSync(srcPath) && fs.statSync(srcPath).isDirectory();
}

/**
 * Gets all packages in the workspace
 */
export function getPackages(workspacePath: string): string[] {
    const fs = require('fs');
    const path = require('path');

    const srcPath = path.join(workspacePath, 'src');
    if (!fs.existsSync(srcPath)) {
        return [];
    }

    const packages: string[] = [];
    const entries = fs.readdirSync(srcPath, { withFileTypes: true });

    for (const entry of entries) {
        if (entry.isDirectory()) {
            const packageXmlPath = path.join(srcPath, entry.name, 'package.xml');
            if (fs.existsSync(packageXmlPath)) {
                packages.push(entry.name);
            }
        }
    }

    return packages;
}

/**
 * Validates package name
 */
export function isValidPackageName(name: string): boolean {
    // ROS 2 package names should only contain lowercase letters, numbers, and underscores
    const regex = /^[a-z][a-z0-9_]*$/;
    return regex.test(name);
}

/**
 * Validates node name
 */
export function isValidNodeName(name: string): boolean {
    // Node names should be valid identifiers
    const regex = /^[a-zA-Z_][a-zA-Z0-9_]*$/;
    return regex.test(name);
}
