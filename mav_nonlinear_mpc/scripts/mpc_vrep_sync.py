from vrep_tests import collisionTest as testLib

class TestOpenCloseVrep(unittest.TestCase):
    
    def setUp(self):
        self.roscore = startRoscore.Roscore()
        self.roscore.run()

    def tearDown(self):
        self.roscore.terminate()
        #killProcess.killProcessByName('vrep')
        
    def test_collision(self):
        tester = testLib.tester()
        testLib.runVrep(tester)
        self.assertEqual(tester.retVal, 0)

if __name__ == '__main__':
    unittest.main()
