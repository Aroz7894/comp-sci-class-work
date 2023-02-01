""" This module provides classes that model shopping carts. 

Author: Andrew Rozniakowski
Version: 1/23/15

Acknowledement: The original version of this code was taken from the
                CodeAcademy's class tutorial: 
 http://www.codecademy.com/courses/python-intermediate-en-WL8e4/\
0/1?curriculum_id=4f89dab3d788890003000096
"""
class ShoppingCart(object):

    """Creates shopping cart objects
    for users of our fine website."""
    def __init__(self, customer_name):
        self.customer_name = customer_name
        self.items_in_cart = {}

    def add_item(self, product, price):
        """Add product to the cart."""
        if not product in self.items_in_cart:
            self.items_in_cart[product] = price
            print product + " added."
        else:
            print product + " is already in the cart."

    def remove_item(self, product):
        """Remove product from the cart."""
        if product in self.items_in_cart:
            del self.items_in_cart[product]
            print product + " removed."
        else:
            print product + " is not in the cart."

    def total_cost(self):
        """Total price of all items in the cart"""
        self.sum_cart = 0 
        for i in self.items_in_cart:
            self.sum_cart += self.items_in_cart[i]
        return self.sum_cart    

class SmallCart(ShoppingCart):
    
    def add_item(self, product, price):
        """Add product to the cart."""
        if len(self.items_in_cart) >= 3:
            print "Cart is Full"
        else:       
            if not product in self.items_in_cart:
                self.items_in_cart[product] = price
                print product + " added."
            else:
                print product + " is already in the cart."
        
def main():
    pass

if __name__ == "__main__":
    main()
