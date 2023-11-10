# 测试 的作用:
# 当完成一个功能的构建后, 又想在此基础上进行改进时
# 要确定增加的改动不会对现有功能产生破坏或负面影响
# 所以就可以对添加对现有功能的测试

from typing import Any
import unittest
from anonymous_survey import AnonymousSurvey

class TestAnonymousSurvey( unittest.TestCase ) :
    '''针对AnonymousSurvey类的测试'''
    def test_store_single_response( self ) :
        '''测试单个答案是否会被妥善存储'''
        question = "What language did you first learn to speak?"
        my_survey = AnonymousSurvey( question )
        my_survey.store_response( 'English' )
        self.assertIn('English', my_survey.responses)
        
    def test_store_three_responses( self ) :
        '''测试三个数据是否会别妥善地存储'''
        question = "What language did you first learn to speak?"
        my_survey = AnonymousSurvey( question )
        responses = [ 'English', 'Spanish', 'Mandarin' ]
        for response in responses :
            my_survey.store_response( response )
            
        for response in responses :
            self.assertIn(response, my_survey.responses)
    
    
if __name__ == '__main__' :
    unittest.main()
