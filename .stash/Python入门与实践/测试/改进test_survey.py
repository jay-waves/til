# 使用setUp()方法. python会先运行它, 然后再运行test_打头的方法
# 每次编写测试方法时, 都可以使用setup提前创建对象, 避免重复定义

# ? 突然发现, 测试自带方法都是驼峰法命名的
import unittest
from anonymous_survey import AnonymousSurvey

class TestAnonymousSurvey(unittest.TestCase) :
    
    def setUp(self) :
        question = "What language did you first learn to speak?"
        self.my_survey = AnonymousSurvey( question )
        self.responses = ['English', 'Spanish', 'Mandarin']
        
    def test_store_single_response( self ) :
        self.my_survey.store_response(self.responses[ 0 ])
        self.assertIn(self.responses[ 0 ], self.my_survey.responses)

    def test_store_three_responses( self ) :
        for response in self.responses :
            self.my_survey.store_response(response)
        for response in self.responses :
            self.assertIn(response, self.my_survey.responses)

if __name__ == '__main__' :
    unittest.main() 
    
# * 时刻为重要的函数或方法编写测试, 可以确保自己所作的工作不会破坏其他项目
# * 这样就不必瞻前顾后, 原功能受损你可以马上知道