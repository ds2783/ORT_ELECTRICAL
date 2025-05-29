

storage_address = ''

from pyzbar.pyzbar import decode
from PIL import Image
import os
output = open("output.txt","w")
for file in os.listdir(storage_adress): 
    # Check whether file is in text format or not 
    if file.endswith(".jpeg"): 
        for qr_code in decode(Image.open(storage_adress+'/'+file)):
            output.write(str(qr_code.data) + str(qr_code.polygon) +"\n")
        
output.close()