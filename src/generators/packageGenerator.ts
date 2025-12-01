import * as fs from 'fs';
import * as path from 'path';

export class PackageGenerator {
    /**
     * Generate a ROS 2 package
     */
    async generatePackage(
        workspaceRoot: string,
        packageName: string,
        buildType: 'ament_cmake' | 'ament_python',
        dependencies: string[],
        description: string
    ): Promise<void> {
        const packagePath = path.join(workspaceRoot, 'src', packageName);

        // Check if package already exists
        if (fs.existsSync(packagePath)) {
            throw new Error(`Package '${packageName}' already exists`);
        }

        // Create package directory
        fs.mkdirSync(packagePath, { recursive: true });

        // Generate package.xml
        this.generatePackageXml(packagePath, packageName, buildType, dependencies, description);

        if (buildType === 'ament_cmake') {
            // Create C++ package structure
            this.generateCMakePackage(packagePath, packageName, dependencies);
        } else {
            // Create Python package structure
            this.generatePythonPackage(packagePath, packageName, dependencies);
        }
    }

    private generatePackageXml(
        packagePath: string,
        packageName: string,
        buildType: string,
        dependencies: string[],
        description: string
    ): void {
        const dependencyTags = dependencies.map(dep => `  <depend>${dep}</depend>`).join('\n');

        const content = `<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${packageName}</name>
  <version>0.0.1</version>
  <description>${description}</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>${buildType}</buildtool_depend>

${dependencyTags}

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>${buildType}</build_type>
  </export>
</package>
`;

        fs.writeFileSync(path.join(packagePath, 'package.xml'), content);
    }

    private generateCMakePackage(packagePath: string, packageName: string, dependencies: string[]): void {
        // Create directories
        fs.mkdirSync(path.join(packagePath, 'src'), { recursive: true });
        fs.mkdirSync(path.join(packagePath, 'include', packageName), { recursive: true });

        // Generate CMakeLists.txt
        const findPackages = dependencies.map(dep => `find_package(${dep} REQUIRED)`).join('\n');
        const dependencies_list = dependencies.length > 0 ? dependencies.join('\n  ') : '';

        const cmakeContent = `cmake_minimum_required(VERSION 3.8)
project(${packageName})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
${findPackages}

# Add executables here
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node
#   ${dependencies_list ? dependencies_list : 'rclcpp'}
# )

# Install targets
# install(TARGETS
#   my_node
#   DESTINATION lib/\${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
`;

        fs.writeFileSync(path.join(packagePath, 'CMakeLists.txt'), cmakeContent);
    }

    private generatePythonPackage(packagePath: string, packageName: string, dependencies: string[]): void {
        // Create directories
        const pythonPackagePath = path.join(packagePath, packageName);
        fs.mkdirSync(pythonPackagePath, { recursive: true });
        fs.mkdirSync(path.join(packagePath, 'resource'), { recursive: true });
        fs.mkdirSync(path.join(packagePath, 'test'), { recursive: true });

        // Create __init__.py
        fs.writeFileSync(path.join(pythonPackagePath, '__init__.py'), '');

        // Create resource marker
        fs.writeFileSync(path.join(packagePath, 'resource', packageName), '');

        // Generate setup.py
        const setupContent = `from setuptools import find_packages, setup

package_name = '${packageName}'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='${packageName} package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'my_node = ${packageName}.my_node:main',
        ],
    },
)
`;

        fs.writeFileSync(path.join(packagePath, 'setup.py'), setupContent);

        // Generate setup.cfg
        const setupCfgContent = `[develop]
script_dir=$base/lib/${packageName}
[install]
install_scripts=$base/lib/${packageName}
`;

        fs.writeFileSync(path.join(packagePath, 'setup.cfg'), setupCfgContent);
    }
}
