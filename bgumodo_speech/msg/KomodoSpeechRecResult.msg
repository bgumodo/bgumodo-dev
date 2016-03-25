Header header
bool success               # Did the recognition command complete successfully

# Recognized phrases
uint8 PHRASE_NULL=255      # Couldn't indentify phrase

uint8 PHRASE_AT_FLOOR_1=1     
uint8 PHRASE_AT_FLOOR_2=2
uint8 PHRASE_AT_FLOOR_3=3
uint8 PHRASE_AT_FLOOR_4=4
uint8 PHRASE_AT_FLOOR_5=5

uint8 PHRASE_YES=6
uint8 PHRASE_NO=7

uint8 PHRASE_GET_COFFEE=8
uint8 PHRASE_MOVE_FORWARD=9
uint8 PHRASE_HERE_YOU_GO=10

uint8 phrase_id               # ID of recognized phrase. 
string phrase               # If phrase_id=PHRASE_NULL, this will hold whatever was recognized (if any) as string
