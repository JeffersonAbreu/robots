import qrcode
import os
'''
pip install qrcode
'''

imagem = qrcode.make("1")
imagem.save("./imgs/1.png")