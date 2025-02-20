from taskbot.msg import BoolOrNull

def boolToBoolOrNull(boolean: bool) -> BoolOrNull:
    if boolean == True:
        return BoolOrNull.TRUE
    elif boolean == False:
        return BoolOrNull.FALSE
    else:
        return BoolOrNull.NULL