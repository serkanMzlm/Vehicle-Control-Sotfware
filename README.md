# Caption

## Code Inspection 

- **`.clang-format`:** Standardizes the appearance of the code. 

```bash
# Displays the formatted code in the terminal without modifying the file
clang-format main.cpp

# Checks for formatting errors without applying any changes.
clang-format --dry-run main.cpp

# Formats the file directly and saves the changes (In-place).
clang-format -i main.cpp
```

- **`.clang-tidy`:** Checks code quality and detects errors.

```bash
clang-tidy main.cpp -- -std=c++17
```