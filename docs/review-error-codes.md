# Review Error Codes

From `context/Error Code Key.xlsx`. Severity: minor (m), normal (n), severe (s).

## Code review (e*)

| Code | Description | m | n | s |
|------|-------------|---|---|---|
| e01 | Unrelated grouped classes in header | 0 | 1 | 2 |
| e02 | Too granular header separation | 0 | 0 | 1 |
| e03 | Unwrapping mp-units types for math | 0 | 0 | 2 |
| e04 | Incorrect use of C++ stdlib | 0 | 1 | 3 |
| e05 | Passing diagnostic message into handler | 0 | 1 | 1 |
| e06 | const-ref params passed as mutable-ref/copy | 0 | 1 | 2 |
| e07 | Class methods could be const | 0 | 1 | 2 |
| e08 | Unneeded methods / data in class API | 0 | 1 | 2 |
| e09 | Too long functions | 0 | 1 | 2 |
| e10 | Duplicate class, functions, or flows | 0 | 1 | 3 |
| e11 | Library requires manual install | 1 | 2 | 4 |
| e13 | Incorrect use of pointers | 0 | 1 | 2 |
| e14 | Incomplete class diagram in HLD | 0 | 1 | 3 |
| e15 | Incomplete sequence diagram in HLD | 0 | 1 | 3 |
| e16 | Numerical fields instead of strong types | 0 | 1 | 2 |
| e17 | Unnecessary dependency | 0 | 1 | 3 |
| e18 | Noncompliant submission | 0 | 2 | 4 |
| e21 | Repeated computation | 0 | 1 | 3 |
| e22 | Poor encapsulation | 1 | 1 | 2 |
| e23 | Magic numbers | 0 | 1 | 1 |

## Program run (b*)

| Code | Description | m | n | s |
|------|-------------|---|---|---|
| b01 | Build failure of main | 1 | 3 | 5 |
| b02 | Build failure of extra | 1 | 2 | 3 |
| b03 | Missing scenario validation | 1 | 1 | 1 |
| b04 | Error on scenario | 3 | 10 | 30 |
| b05 | Timeout on scenario (1 min) | 2 | 5 | 20 |
| b06 | No error on missing input | 1 | 1 | 1 |
| b07 | Manual config change failure | 5 | 10 | 10 |
| b08 | Scenario required manual change | 5 | 5 | 5 |

## Review guideline summary

- Happy flow completes on default scenarios (also from alternate path, minor valid config edits)
- Invalid config exits gracefully
- Reasonable runtime for all cases
- Coherent structure; clean include DAG
- const-correctness, encapsulation, lean classes
- No raw new/delete/malloc; smart pointers + RAII
- No magic numbers; no expensive recomputation
- Short functions, single responsibility
- HLD and readme match code
