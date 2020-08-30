import unittest

if __name__ == '__main__':
    suite = unittest.TestLoader().discover("./tests/",pattern='test*.py')
    unittest.TextTestRunner(verbosity=2).run(suite)
