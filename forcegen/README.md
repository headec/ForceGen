# omega
## x,y,z limits (moved by hand)
-0.049 +0.073
-0.111 +0.111
-0.103 +0.122

## x,y,z limits (using drd functions assuming other axises are 0) 
-0.045 +0.073
-0.093 +0.093
-0.090 +0.105

## x limit when drawing a circle(max y or z == 0.08(m; radius))
-0.047 +0.021

## to build aging test
cd build/ <br>
./fd-build ${OUTPUT_FILE_NAME}  // file name is optional; default: pre_aging

## to build r/w test
cd build/ <br>
./fd-build ${OUTPUT_FILE_NAME} read_write_test

## to build custom cpp files
cd build/ <br>
./fd-build ${OUTPUT_FILE_NAME} ${CUSTOM_FILE_NAME}
