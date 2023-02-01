package db;

import java.io.UnsupportedEncodingException;
import java.net.URLEncoder;
import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;

/**
 * A row of the car table.
 *
 * @author Chris Mayfield
 * @version 03/25/2015
 */
public class Car {

    public String vin;
    public String make;
    public String model;
    public int miles;
    public float price;

    /**
     * Constructs a car from the given values.
     */
    public Car(String vin, String make, String model, int miles, float price) {
        this.vin = vin;
        this.make = make;
        this.model = model;
        this.miles = miles;
        this.price = price;
    }

    /**
     * Inserts the car into the database.
     */
    public void insert() throws SQLException {
        Connection db = Database.open();
        PreparedStatement st = db.prepareStatement(
                "INSERT INTO car VALUES (?, ?, ?, ?, ?)");
        st.setString(1, vin);
        st.setString(2, make);
        st.setString(3, model);
        st.setInt(4, miles);
        st.setFloat(5, price);
        st.executeUpdate();
        st.close();
        db.close();
    }

    /**
     * Creates a link to delete this car.
     */
    public String delLink() {
        StringBuilder str = new StringBuilder();
        str.append("<a href=\"index.jsp?del=");
        try {
            str.append(URLEncoder.encode(vin, "UTF-8"));
        } catch (UnsupportedEncodingException exc) {
            exc.printStackTrace();
        }
        str.append("\">Delete</a>");
        return str.toString();
    }

    /**
     * Deletes a car from the database.
     */
    public static void delete(String vin) throws SQLException {
        Connection db = Database.open();
        PreparedStatement st = db.prepareStatement(
                "DELETE FROM car WHERE vin = ?");
        st.setString(1, vin);
        st.executeUpdate();
        st.close();
        db.close();
    }

    /**
     * Returns all the cars in the database.
     */
    public static ArrayList<Car> selectAll() throws SQLException {
        Connection db = Database.open();
        Statement st = db.createStatement();
        ResultSet rs = st.executeQuery("SELECT * FROM car");
        ArrayList<Car> list = new ArrayList<Car>();
        while (rs.next()) {
            String vin = rs.getString(1);
            String make = rs.getString(2);
            String model = rs.getString(3);
            int miles = rs.getInt(4);
            float price = rs.getFloat(5);
            Car car = new Car(vin, make, model, miles, price);
            list.add(car);
        }
        rs.close();
        st.close();
        db.close();
        return list;
    }

}
