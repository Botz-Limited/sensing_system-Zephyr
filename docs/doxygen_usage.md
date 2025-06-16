# Doxygen Usage Guide

## Introduction

Doxygen is a powerful tool for generating documentation from annotated source code. By following the Doxygen style for code comments, you can produce clear and comprehensive documentation for your software projects. This guide provides instructions on how to use Doxygen to generate documentation using the Doxygen style for code comments.

## Table of Contents

- [Doxygen Usage Guide](#doxygen-usage-guide)
  - [Introduction](#introduction)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
  - [Configuring Doxygen](#configuring-doxygen)
    - [Doxygen Configuration Example](#doxygen-configuration-example)
  - [Doxygen Style for Code Comments](#doxygen-style-for-code-comments)
  - [Generating Documentation](#generating-documentation)
  - [Additional Doxygen Features](#additional-doxygen-features)
  - [Example](#example)
  - [Conclusion](#conclusion)

## Installation

Before using Doxygen, you need to install it on your system. You can download the latest version from the official [Doxygen website](http://www.doxygen.nl/download.html) or use package managers like `apt`  for Linux.

```bash
# Example installation using apt on Ubuntu
sudo apt-get install doxygen
```

## Configuring Doxygen

Create a configuration file named `Doxyfile` in your project directory. You can generate a template using the following command:

```bash
doxygen -g
```

Edit the generated `Doxyfile` to customize settings such as input directories, output format, and documentation style.

### Doxygen Configuration Example

Here's an example of a Doxygen configuration:

```plaintext
# Doxyfile

# Set the input source code directory
INPUT                  = ./src

# Set the output directory for the generated documentation
OUTPUT_DIRECTORY       = ./docs

# Specify file patterns to include in the documentation
FILE_PATTERNS          = *.cpp *.h

# Other configuration settings...
```

Refer to the [Doxygen Manual](http://www.doxygen.nl/manual/config.html) for detailed configuration options.

## Doxygen Style for Code Comments

Doxygen comments are written using a specific syntax to document various elements in your code. The basic structure of a Doxygen comment looks like this:

```cpp
/**
 * Brief description.
 *
 * Detailed description (optional).
 *
 * @param parameter_name Description of the parameter.
 * @return Description of the return value.
 */
```

Use `@brief` for a short description and `@param` and `@return` tags to document function parameters and return values.

## Generating Documentation

Run the following command in your project directory to generate documentation:

```bash
doxygen Doxyfile
```

This will create an `html` or `latex` directory (based on your configuration) containing the generated documentation.

## Additional Doxygen Features

- **File Documentation**: Add a Doxygen comment at the beginning of each source and header file.
- **Grouping**: Use `\defgroup` and `\ingroup` to group related classes, functions, or modules.
- **Cross-Referencing**: Link to other elements using `\ref` or `\link`.

Refer to the [Doxygen Manual](http://www.doxygen.nl/manual/index.html) for a comprehensive list of features.

## Example

```cpp
/**
 * @file main.cpp
 * @author 
 * @brief
 * @version 1.1.1
 * @date 2024-03-11
 *
 * @copyright Botz Innovation 2025
 *
 */

#include <iostream>

/**
 * @brief Adds two numbers.
 *
 * This function adds two numbers and returns the result.
 *
 * @param a The first number.
 * @param b The second number.
 * @return The sum of a and b.
 */
int add(int a, int b) {
    return a + b;
}

int main() {
    int result = add(3, 4);
    std::cout << "Result: " << result << std::endl;
    return 0;
}
```
Refer this [link](https://github.com/cschlosser/doxdocgen?tab=readme-ov-file#auto-complete-doxygen-commands) on how to use this extension


## Conclusion
By following the Doxygen style for code comments, you can enhance the readability of your code and automatically generate comprehensive documentation. Customize the Doxygen configuration to suit your project's needs and regularly update the documentation as your code evolves.
