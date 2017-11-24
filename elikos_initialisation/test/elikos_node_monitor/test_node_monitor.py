import unittest

import elikos_initialisation
import elikos_node_monitor.node_monitor as nm
import sqlite3
import os
        



class TestTest(unittest.TestCase):
    
    def __init__(self, *args, **kwargs):
        super(TestTest, self).__init__(*args, **kwargs);
        self.table_name = "test_table"
        self.db_name = "test_batabase"


    def setUp(self):
        con = sqlite3.connect(self.db_name)

        #create the table
        c = con.cursor()
        c.execute("CREATE TABLE IF NOT EXISTS " + self.table_name + "(status INTEGER, name STRING PRIMARY KEY, pid INTEGER)")
        c.executemany("INSERT INTO {0} VALUES (?,?,?)".format(self.table_name),
            [(nm.ns.RUNNING, "test-ham", 2830),
             (nm.ns.PENDING, "test-hem", 2830),
             (nm.ns.STOPPED, "test-him", 2830),
             (nm.ns.CRASHED, "test-hom", 2830),
             (nm.ns.PENDING, "test-hum", 2830),
             (nm.ns.FAILING, "test-hym", 2830)]
        )
        con.commit()

    def tearDown(self):
        if os.path.isfile(self.db_name):
            os.remove(self.db_name)


    def test_database_wrapper(self):
        db = nm.DBWrapper(self.db_name, self.table_name)
        
        self.assertEqual(db.get_status("test-ham"), nm.ns.RUNNING)
        self.assertEqual(db.get_status("test-hum"), nm.ns.PENDING)

        db.set_status("test-ham", nm.ns.FAILING)

        self.assertEqual(db.get_status("test-ham"), nm.ns.FAILING)

        del db
        #os.remove(db_name)

    def test_some_other_func(self):
        pass


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('elikos_initialisation', 'test_node_monitor', TestTest)
