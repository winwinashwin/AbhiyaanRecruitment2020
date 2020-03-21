import json
import os


# setup subscriber database
def initialise():
    if not os.path.exists(".\\subscribers"):
        os.mkdir(".\\subscribers")
    os.chdir(".\\subscribers")


class Subscriber:
    def __init__(self, name, mstr):
        self.master = mstr  # to refer to functions in master
        self.name = name
        self.topics = []  # list of subscribed topics
        self.filename = self.name + ".json"
        if not os.path.exists(self.filename):
            open(self.filename, "w").close()  # setup database
            self.database = {}
        else:
            self.database = json.loads(open(self.filename).read())  # read existing data

    # method to subscribe to different topics
    def subscribe_to(self, *topics):
        for topic in topics:
            if topic in self.topics:  # check if already subscribed
                print(f">>> Subscriber {self.name} is already subscribed to topic : {topic}.")
            else:
                self.topics.append(topic)  # add topic to list of subscribed topics
                print(f">>> {self.name} has successfully subscribed to topic : {topic}.")
            # add subscriber object to list of subscribers subscribed to a topic in master
            if topic not in self.master.topics:
                self.master.topics[topic] = [self]
            else:
                self.master.topics[topic].append(self)

    # method to unsubscribe from topic/topics
    def unsubscribe_from(self, *topics):
        for topic in topics:
            # check if subscribed
            if topic not in self.topics:
                print(f">>> Subscriber {self.name} is not subscribed to topic : {topic}")
            else:
                self.topics.remove(topic)  # remove topic from list of subscribed topics
                self.master.topics[topic].remove(self)  # remove subscriber from subscription to the topic in master
                print(f">>> {self.name} has successfully unsubscribed from topic : {topic}")

    # to save received data to database
    def _save_received_data(self, data):
        topic = data['topic']
        string = data['string']
        if topic in self.database:
            self.database[topic].append(string)
        else:
            self.database[topic] = [string]
        with open(self.filename, "w") as fp:
            fp.write(json.dumps(self.database, indent=4))


class Publisher:
    def __init__(self, name, mster):
        self.name = name
        self.__master = mster  # pointer to master for accessing functions in master

    def publish(self, *args):
        self.__master._new_content(*args)  # publish content to master


class Master:
    def __init__(self):
        self.subscribers = {}  # dictionary to hold subscriber objects with name as key
        self.publishers = {}  # dictionary to hold publisher objects with name as key
        self.topics = {}  # dictionary to hold subscribers subscribed to a topic(key)

    # method to add new subscriber
    def new_subscriber(self, name):
        # check if already subscribed
        if name in self.subscribers:
            print(f">>> Subscriber {name} already exists.")
        else:
            self.subscribers[name] = Subscriber(name, self)  # add subscription
            print(f">>> Successfully added Subscriber {name}.")

    # method to remove subscriber
    def remove_subscriber(self, name):
        # validate subscription
        if name not in self.subscribers:
            print(f">>> Subscriber {name} does not exist.")
        else:
            for topic in self.topics:
                # remove subscription from each topic if subscribed
                if self.subscribers[name] in self.topics[topic]:
                    self.topics[topic].remove(self.subscribers[name])
            # remove subscriber from subscribers
            del self.subscribers[name]
            print(f">>> Successfully removed Subscriber : {name}")

    # method to add publisher
    def add_publisher(self, name):
        # check if publisher already exists
        if name in self.publishers:
            print(f">>> Publisher {name} already exists.")
        else:
            self.publishers[name] = Publisher(name, self)  # add publisher
            print(f">>> Successfully added Publisher {name}.")

    # method to remove publisher
    def remove_publisher(self, name):
        # check if publisher exists
        if name not in self.publishers:
            print(f">>> Publisher {name} does not exist.")
        else:
            # remove publisher from list of publishers
            del self.publishers[name]
            print(f">>> Successfully removed Publisher : {name}")

    # method to send data-string to relevant subscribers
    def _new_content(self, *database):
        for data in database:
            if data['topic'] in self.topics:
                for subscription in self.topics[data['topic']]:
                    subscription._save_received_data(data)


if __name__ == "__main__":
    initialise()

    master = Master()

    # the following is a sample code to convey usage of the methods in the main function
    master.new_subscriber('sub1')
    master.subscribers['sub1'].subscribe_to('plants')

    master.new_subscriber('sub2')
    master.subscribers['sub2'].subscribe_to('animals')

    master.new_subscriber('sub3')
    master.subscribers['sub3'].subscribe_to('plants', 'birds')

    master.add_publisher('pub1')
    master.add_publisher('pub2')

    data1 = {
        'topic': 'plants',
        'string': 'this is a string on plants.'
    }
    data2 = {
        'topic': 'birds',
        'string': 'this is a string on birds.'
    }
    data3 = {
        'topic': 'animals',
        'string': 'this is a string on animals.'
    }
    data4 = {
        'topic': 'plants',
        'string': 'this is second string on plants'
    }

    master.publishers['pub1'].publish(data1, data2)
    master.publishers['pub2'].publish(data3, data4)

    master.remove_subscriber('sub2')
    master.remove_publisher('pub2')
