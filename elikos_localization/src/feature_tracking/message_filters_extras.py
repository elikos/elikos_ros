import message_filters


class Combiner(message_filters.SimpleFilter):
    """
    source gives a lot of messages. The combiner combines them all into a tuple.
    """
    def __init__(self, source, combiner_function):
        message_filters.SimpleFilter.__init__(self)
        self.source = source
        self.connection = source.registerCallback(self.passMessage)
        self.combiner_function = combiner_function

    def passMessage(self, *args):
        self.signalMessage(self.combiner_function(*args))
