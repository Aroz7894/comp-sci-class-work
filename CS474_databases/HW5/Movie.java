package db;

import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;

/**
 * Represents a movie in IMDb.
 *
 * @author Chris Mayfield
 */
public class Movie {

    public int id;
    public String title;
    public int year;
    public ArrayList<String> genres;

    /**
     * Constructs a Movie from the database.
     *
     * @param id identifies the movie
     */
    public Movie(int id) throws SQLException {
        String sql;
        PreparedStatement st;
        ResultSet rs;
        Connection db = Database.open();

        // get the movie information
        sql = "SELECT title, year FROM movie WHERE id = ?";
        st = db.prepareStatement(sql);
        st.setInt(1, id);
        rs = st.executeQuery();
        if (!rs.next()) {
            throw new IllegalArgumentException("invalid movie id");
        }
        this.id = id;
        this.title = rs.getString(1);
        this.year = rs.getInt(2);
        rs.close();

        // get the movie's genres
        this.genres = new ArrayList<String>();
        sql = "SELECT info FROM movie_info WHERE movie_id = ? AND info_id = 3";
        st = db.prepareStatement(sql);
        st.setInt(1, id);
        rs = st.executeQuery();
        while (rs.next()) {
            this.genres.add(rs.getString(1));
        }
        rs.close();
        st.close();
        db.close();
    }

    /**
     * Inserts this movie into the database.
     */
    public void insert() throws SQLException {
        // hint: PreparedStatement and executeUpdate
    }

    /**
     * Deletes this movie from the database.
     */
    public void delete() throws SQLException {
        // hint: PreparedStatement and executeUpdate
    }

}
