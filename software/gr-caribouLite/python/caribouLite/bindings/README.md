# GNU Radio Python bindings

This directory contains the native source block's Python bindings and docstring
templates. The parent module conditionally includes Python/GRC support through
`ENABLE_PYTHON`; see the [module build](../../../CMakeLists.txt).

The presence of bindings does not establish that a particular GNU Radio/Python
combination builds or installs correctly. Packaging and an import/GRC smoke test
remain [DOC-07](../../../../../ROADMAP.md#documentation-validation-backlog).
