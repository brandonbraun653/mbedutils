These files were copied from the flashdb library. When using include paths for cmake targets, the
"fdb_cfg.h" file packaged with the library was being found before the project's custom header, making
for a large number of compilation issues. This gets around it by providing an alternative location
for these files to live.