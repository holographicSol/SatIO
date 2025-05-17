-----

[ Add New Matrix Function ]

1: Optional: Create new variable(s) (related xyz) as required.

2: CTRL+F: MatrixStruct -> add to matrix_function_names.

3: CTRL+F: MatrixStruct -> increase max_matrix_function_names.

4: CTRL+F: menuMatrixSetFunctionNameItems -> increase max and items.

5: CTRL+F: void matrixSwitch() -> express switching conditions for associated values.

6: CTRL+F: getRelatedX() -> add as required.

7: CTRL+F: getRelatedY() -> add as required.

8: CTRL+F: getRelatedZ() -> add as required.

The new function should now show up in available functions and can be used programmably 
with expressions (over, under, equal, range) and can be saved/loaded from matrix files.

-----

[ Add New Menu Page ]

1: CTRL+F: DISPLAY VARIABLES -> add page number and menu as required.

2: CTRL+F: void menuBack() -> add page as required.

3: CTRL+F: void menuUp() -> add page as required.

4: CTRL+F: void menuDown() -> add page as required.

5: CTRL+F: void menuEnter() -> add page as required.

6: CTRL+F: void void UpdateUI(void * pvParamters) add page as required.

The new menu page should now show up in the UI and be fully functional.

-----

[ Add New System Configuration Value ]

1: CTRL+F: systemStruct -> add value(s) as required.

2: CTRL+F: void sdcardSaveSystemConfig(char * file) -> add as required.

3: CTRL+F: sdcardLoadSystemConfig(char * file) -> add as required.

The new value will be loaded every startup after system configuration has been saved.

-----
