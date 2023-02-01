<%@page import="java.util.*, db.*" %>
<!DOCTYPE html>
<html>
    <head>
        <meta charset="UTF-8">
        <title>Cars!</title>
    </head>
    <body>
        <%
        // perform the requested actions
        if (request.getParameter("ins") != null) {
            String vin = request.getParameter("vin");
            String make = request.getParameter("make");
            String model = request.getParameter("model");
            int miles = Integer.parseInt(request.getParameter("miles"));
            float price = Float.parseFloat(request.getParameter("price"));
            Car car = new Car(vin, make, model, miles, price);
            car.insert();
        }
        if (request.getParameter("del") != null) {
            String vin = request.getParameter("del");
            Car.delete(vin);
        }
        %>
        <h3>Cars!</h3>
        <form>
            <table border="1" cellpadding="3">
                <tr>
                    <th>VIN</th>
                    <th>Make</th>
                    <th>Model</th>
                    <th>Miles</th>
                    <th>Price</th>
                    <th>&nbsp;</th>
                </tr>
                <%
                // display the table of cars
                ArrayList<Car> cars = Car.selectAll();
                for (Car car : cars) {
                    out.println("<tr>");
                    out.println("<td>" + car.vin + "</td>");
                    out.println("<td>" + car.make + "</td>");
                    out.println("<td>" + car.model + "</td>");
                    out.println("<td>" + car.miles + "</td>");
                    out.println("<td>" + car.price + "</td>");
                    out.println("<td>" + car.delLink() + "</td>");
                    out.println("</tr>");
                }
                %>
                <tr>
                    <td><input type="text" name="vin"></td>
                    <td><input type="text" name="make"></td>
                    <td><input type="text" name="model"></td>
                    <td><input type="text" name="miles"></td>
                    <td><input type="text" name="price"></td>
                    <td><input type="submit" name="ins" value="Insert" />
                </tr>
            </table>
        </form>
    </body>
</html>
