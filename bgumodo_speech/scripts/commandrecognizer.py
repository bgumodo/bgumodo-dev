import re

def recognize_command(recognized_text):
    match = re.search('KOMODO.*COFFEE', recognized_text)
    if match:
        return 'coffee'
    else:
        return None

def main():
    recognized_text = ' '.join(['<s>', 'KOMODO(2)', 'GO', 'BRING', 'ME', 'COFFEE', '</s>'])
    print(recognize_command(recognized_text))

if __name__ == '__main__':
    main()
