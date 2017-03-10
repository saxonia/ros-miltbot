$(function() {
   $('#nav a').click(function() {
      $('#nav a').removeClass("is-active");
     //  $($(this).attr('href')).addClass('is-active');
      $($(this)).addClass('is-active');
   });
});

// var cars = ["BMW", "Volvo", "Saab", "Ford", "Fiat", "Audi"];
// for (i = 0; i < cars.length; i++) {
//     text += cars[i] + "<br>";
// }
// document.getElementById("demo").innerHTML = text;