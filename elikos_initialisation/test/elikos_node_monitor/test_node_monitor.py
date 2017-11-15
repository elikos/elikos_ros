import unittest

import elikos_initialisation



class TestTest(unittest.TestCase):
    def test_database_wrapper(self):
        import elikos_node_monitor.node_monitor as nm
        import sqlite3
        import os
        print os.path.abspath('.')

        table_name = "some_name"
        db_name = "test.db"

        con = sqlite3.connect(db_name)

        #create the table
        c = con.cursor()
        c.execute("CREATE TABLE IF NOT EXISTS " + table_name + "(status INTEGER, name STRING PRIMARY KEY, pid INTEGER)")
        c.executemany("INSERT INTO {0} VALUES (?,?,?)".format(table_name),
            [(nm.ns.RUNNING, "test-ham", 2830),
             (nm.ns.PENDING, "test-hem", 2830),
             (nm.ns.STOPPED, "test-him", 2830),
             (nm.ns.CRASHED, "test-hom", 2830),
             (nm.ns.PENDING, "test-hum", 2830),
             (nm.ns.FAILING, "test-hym", 2830)]
        )
        con.commit()

        db = nm.DBWrapper(db_name, table_name)
        
        self.assertEqual(db.get_status("test-ham"), nm.ns.RUNNING)
        self.assertEqual(db.get_status("test-hum"), nm.ns.PENDING)

        db.set_status("test-ham", nm.ns.FAILING)

        self.assertEqual(db.get_status("test-ham"), nm.ns.FAILING)

        del db
        #os.remove(db_name)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('elikos_initialisation', 'test_node_monitor', TestTest)
