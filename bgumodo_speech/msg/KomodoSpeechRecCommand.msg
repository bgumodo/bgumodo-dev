Header header

# Command meta-data
# String value for start command 
string CMD_START=start
# String value for stop command
string CMD_STOP=stop             

string cmd     # Command type. Values are either CMD_START or CMD_STOP (default).


# Category data (Might aid in speech recognition)
# Possible answer to a "yes/no" question
string CAT_YESNO=yes_no

# Location description (e.g. 5th floor, building 37, etc.)
string CAT_LOCATIOn=location

# Command description (e.g. go get coffee)
string CAT_COMMAND=command

# All known phrases
string CAT_ALL=all               

string cat     # Expected phrase category. Defaults to CAT_ALL
