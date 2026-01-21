# -*- coding: utf-8 -*-
import PyPDF2
import sys
sys.stdout.reconfigure(encoding='utf-8')

pdf_path = r'e:\cursor project\04_21376223_刘丹阳.pdf'
with open(pdf_path, 'rb') as f:
    reader = PyPDF2.PdfReader(f)
    # Read pages 24-35 (0-indexed: 23-34)
    for i in range(23, 35):
        if i < len(reader.pages):
            page = reader.pages[i]
            text = page.extract_text()
            print(f'=== Page {i+1} ===')
            print(text)
            print()
