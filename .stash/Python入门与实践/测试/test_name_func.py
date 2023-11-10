import unittest

from name_function import get_fomatted_name


class TestNameFunc(unittest.TestCase):
    '''测试name_function.py'''

    def test_name_func(self):
        formatted_name = get_fomatted_name('john', 'sarah')
        self.assertEqual(formatted_name, 'John Sarah')

    def test_name_middle_func(self):
        formatted_name = get_fomatted_name('john', 'sarah', 'messi')
        self.assertEqual(formatted_name, 'John Messi Sarah')
    


if __name__ == "__main__":
    unittest.main()
