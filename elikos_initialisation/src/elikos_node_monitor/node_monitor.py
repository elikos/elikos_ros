#!/usr/bin/env python
import sqlite3
import rospy
import node_satus as ns
from elikos_ros.msg import NodeStatus



class DBWrapper(object):
    def __init__(self, db_name, table_name="node_status"):
        u"""
        Open a connection to DB, creates one if it is null. The database should
        never be null, so if it is, there is a problem.
        """
        self.db_name = db_name
        self.table_name = table_name
        self.conn = sqlite3.connect(db_name)
        cursor = self.conn.cursor()

        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='{0}'".format(table_name))
        if len(cursor.fetchall()) < 1:
            rospy.logfatal("No table '{0}' in database!".format(self.table_name))
            cursor.execute("CREATE TABLE " + self.table_name + "(status INTEGER, name STRING, pid INTEGER)")
            self.conn.commit()


    def set_status(self, node_name, new_status):
        cursor = self.conn.cursor()
        cursor.execute(
            "UPDATE {0} SET status=? WHERE name=?".format(self.table_name),
            (new_status, node_name)
        )
        self.conn.commit()


    def get_status(self, node_name):
        cursor = self.conn.cursor()
        cursor.execute("SELECT status FROM " + self.table_name + " WHERE name=?", (node_name,))
        results = cursor.fetchall()
        if len(results) < 1 :
            return None
        else:
            return results[0][0];



def callback(msg, database):
    node_name = msg.node_name
    new_status = msg.new_status
    old_status = database.get_status(node_name)

    if old_status is None:
        rospy.logerr("Node {0} does not exsist in the database.")
        return

    allowed = ns.is_allowed_transition(old_status, new_status)
    if not allowed:
        rospy.logerr("Node {0} requested unallowed transition from {1} to {2}".format(
            node_name,
            ns.to_string(old_status),
            ns.to_string(new_status)
        ))
        return

    database.set_status(node_name, new_status)






if __name__ == "__main__":
    rospy.init_node("db_error_logger");# we need roscore
    db = DBWrapper("node_status.db");
    rospy.Subscriber(
        "node_status",
        NodeStatus,
        callback=callback,
        callback_args=db,
        queue_size=None
    )

    pub = rospy.Publisher(
        "node_status",
        NodeStatus
    )

    n = NodeStatus()
    n.node_name = rospy.get_name()
    n.status = ns.RUNNING
    pub.publish(n)

    rospy.spin()



