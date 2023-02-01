<?php

/**
 * Represents a movie in IMDb.
 */
class Movie {

    public $id;
    public $title;
    public $year;
    public $genres;

    /**
     * Selects a Movie from the database.
     */
    public function __construct($id) {
        $con = Database::open();

        // get the movie information
        $sql = 'SELECT title, year FROM movie WHERE id = $1';
        pg_prepare($con, "", $sql);
        $rs = pg_execute($con, "", array($id))
                or die("Query failed: " . pg_last_error());
        $row = pg_fetch_row($rs);
        $this->id = $id;
        $this->title = $row[0];
        $this->year = $row[1];

        // get the movie's genres
        $this->genres = array();
        $sql = 'SELECT info FROM movie_info WHERE movie_id = $1 AND info_id = 3';
        pg_prepare($con, "", $sql);
        $rs = pg_execute($con, "", array($id))
                or die("Query failed: " . pg_last_error());
        while ($row = pg_fetch_row($rs)) {
            array_push($this->genres, $row[0]);
        }
        pg_close($con);
    }

    /**
     * Inserts this movie into the database.
     */
    public function insert() {
        // hint: pg_prepare and pg_execute
    }

    /**
     * Deletes this movie from the database.
     */
    public function delete() {
        // hint: pg_prepare and pg_execute
    }

}
