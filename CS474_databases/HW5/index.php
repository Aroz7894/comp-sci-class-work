<?php
include "Database.php";
include "Movie.php";
?>
<!DOCTYPE html>
<html>
    <head>
        <meta charset="UTF-8">
        <title>IMDB Example</title>
    </head>
    <body>
        <h2>IMDB Example</h2>
        <form>
            <?php
            if (empty($_REQUEST["movie_id"])) {
                $str = "";
            } else {
                $str = $_REQUEST["movie_id"];
            }
            ?>
            Enter a movie id:
            <input type="text" name="movie_id" value="<?php echo $str; ?>">
            <input type="submit">
            <br>
            <?php
            // only run the query if the form was submitted
            if (!empty($str)) {
                $id = intval($str);
                $movie = new Movie($id);
                ?>
                <p><b>Title:</b><br> <?php echo $movie->title; ?></p>
                <p><b>Year:</b><br>  <?php echo $movie->year; ?></p>
                <p><b>Genres:</b></p>
                <ul>
                    <?php
                    foreach ($movie->genres as $genre) {
                        echo "<li>" . $genre . "</li>";
                    }
                    ?>
                </ul>
                <?php
            }
            ?>
        </form>
    </body>
</html>
