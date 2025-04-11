# C++/ROS 2 Utilities Utilities
[![Build](https://github.com/maximilian-nitsch/Navigation-Utilities/actions/workflows/ci.yaml/badge.svg)](https://github.com/maximilian-nitsch/Navigation-Utilities/actions)
[![License](https://img.shields.io/github/license/maximilian-nitsch/Navigation-Utilities.svg)](https://github.com/maximilian-nitsch/Navigation-Utilities/blob/main/LICENSE)
[![Last Commit](https://img.shields.io/github/last-commit/maximilian-nitsch/Navigation-Utilities)](https://github.com/maximilian-nitsch/Navigation-Utilities/commits/main)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://index.ros.org/doc/ros2/Installation/Humble/)
[![Release](https://img.shields.io/github/v/release/maximilian-nitsch/Navigation-Utilities)](https://github.com/maximilian-nitsch/Navigation-Utilities/releases)
[![Open Issues](https://img.shields.io/github/issues/maximilian-nitsch/Navigation-Utilities)](https://github.com/maximilian-nitsch/Navigation-Utilities/issues)
[![Contributors](https://img.shields.io/github/contributors/maximilian-nitsch/Navigation-Utilities)](https://github.com/maximilian-nitsch/Navigation-Utilities/graphs/contributors)

<!--- protected region package header begins -->
**Author:**
- Maximilian Nitsch <maximilian.nitsch@irt.rwth-aachen.de>

**Affiliation:** Institute of Automatic Control - RWTH Aachen University

**Maintainer:**
  - Maximilian Nitsch <maximilian.nitsch@irt.rwth-aachen.de>
<!--- protected region package header ends -->

## Description
Dedicated Repo for Navigation Utility Functions.

## Table of Contents

- [Dependencies](#dependencies)
- [Installation](#installation)
- [Contributing](#contributing)
- [License](#license)

# Dependencies

This project depends on:

- **ROS 2 Humble**: ROS 2 is a set of software libraries and tools for building robot applications: [ROS 2 Installation page](https://docs.ros.org/en/humble/Installation.html).

# Installation

To install the navigation utilities, you need to follow these steps:

1. **Install ROS 2 Humble**: Ensure you have ROS 2 (Humble) installed. You can follow the official installation instructions provided by ROS 2. Visit [ROS 2 Humble Installation page](https://docs.ros.org/en/humble/Installation.html) for detailed installation instructions tailored to your platform.

3. **Clone the Package**: Clone the package repository to your ROS 2 workspace. If you don't have a ROS 2 workspace yet, you can create one using the following commands:

    ```bash
    mkdir -p /path/to/ros2_workspace/src
    cd /path/to/ros2_workspace/src
    ```

    Now, clone the package repository:

    ```bash
    git clone <repository_url>
    ```

    Replace `<repository_url>` with the URL of your package repository.

4. **Build the Package**: Once the package is cloned, you must build it using colcon, the default build system for ROS 2. Navigate to your ROS 2 workspace and run the following command:

    ```bash
    cd /path/to/ros2_workspace
    colcon build
    ```

    This command will build all the packages in your workspace, including the newly added package.

5. **Using Navigation-Utilities in Your ROS 2 Package**

    To use the navigation utilities function provided by **Navigation-Utilities** in your own ROS 2 package, follow these steps:
    
    In your package's `package.xml`, add:
    
    ```xml
      <depend>navigation_utilities_package</depend>
     ```
    In your package's `CMakeLists.txt`, add:
    ```xml
    find_package(navigation_utilities_package REQUIRED)
    
      target_link_libraries(...  navigation_utilities_package::navigation_utilities_package)
    ```  
That's it! Your navigation utilities should now be installed and ready to use in your ROS 2 environment.

## Contributing

If you'd like to contribute to the project, please take a look at the [CONTRIBUTING](CONTRIBUTING) file for details.

## License

This project is licensed under the BSD-3-Clause License. Please look at the [LICENSE](LICENSE) file for details.
