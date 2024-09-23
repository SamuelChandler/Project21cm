# Project 21cm 

## To Image executable 
to create the executable from the python file using the pyinstaller tool and using the command below in the base directory

```
pyinstaller ToImage.py --onefile --clean --noconsole --distpath ./Executables
```

## Export Executable Creation
```
pyinstaller Export.py --onefile --clean --noconsole --distpath ./Executables
```


## Libraries Used 
- xlrd
- pyinstaller
- Pillow (PIL)
- Numpy
- pandas
- pyhackrf
- matplotlib
- scipy
- time