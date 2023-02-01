""" Simple unit tests for carts.py. 

Author: Nathan Sprague
Version: 8/26/14
"""

import unittest
import carts

class TestCarts(unittest.TestCase):
    def setUp(self):
        self.cart = carts.ShoppingCart("Jill")
        self.cart.add_item("tree", 2)
        self.cart.add_item("house", 3)
        self.cart.add_item("jump", 7)

        self.small_cart  = carts.SmallCart("Bob")
        self.small_cart.add_item("tree", 2)
        self.small_cart.add_item("house", 3)
        self.small_cart.add_item("jump", 7)


    def test_cart_total_cost(self):
        self.assertEqual(self.cart.total_cost(), 12)

    def test_small_cart_name(self):
        cart = carts.SmallCart("Bob")
        self.assertEqual(self.small_cart.customer_name, "Bob")

    def test_small_cart_remove_item(self):
        self.small_cart.remove_item("house")
        self.assertFalse("house" in self.small_cart.items_in_cart)

    def test_small_cart_add_item_below_limit(self):
        self.assertEqual(self.small_cart.items_in_cart["tree"], 2)
        self.assertEqual(self.small_cart.items_in_cart["house"], 3)
        self.assertEqual(self.small_cart.items_in_cart["jump"], 7)

    def test_small_cart_add_item_fell_below_limit(self):
        self.small_cart.remove_item("house")
        self.small_cart.add_item("pig", 5)
        self.assertEqual(self.small_cart.items_in_cart["pig"], 5)

    def test_small_cart_add_item_above_limit(self):
        self.small_cart.add_item("big", 9)
        self.assertFalse("big" in self.small_cart.items_in_cart)
        
    def test_small_cart_total_cost(self):
        self.assertEqual(self.small_cart.total_cost(), 12)

if __name__ == '__main__':
    unittest.main()
