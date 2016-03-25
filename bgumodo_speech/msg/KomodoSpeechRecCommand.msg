Header header

# Command meta-data
string CMD_START=start           # String value for start command 
string CMD_STOP=stop             # String value for stop command

string cmd                       # Indicate type of command. Value are either CMD_START or CMD_STOP (default).

# Category data (Might aid in speech recognition)
string CAT_YESNO=yes_no          # Possible answer to a "yes/no" question
string CAT_LOCATIOn=location     # Location description (e.g. 5th floor, building 37, etc.)
string CAT_COMMAND=command       # Command description (e.g. go get coffee)
string CAT_ALL=all               # All known phrases

string cat                       # Expected phrase category. Defaults to CAT_ALL
