using System.Net.Http;
using System.Threading.Tasks;
using TMPro;
using System.Text;

public class APIManager
{
    static readonly HttpClient client = new HttpClient();

    public static string robotArmManagerURI = "";

    public string defaultServerURI = "http://10.138.226.61:5002/";

    TMP_Text text;


    public APIManager()
    {
        if (robotArmManagerURI == null || robotArmManagerURI == ""){
            robotArmManagerURI = defaultServerURI;
        }
    }

    async Task PostAsync(string uri, string requestBody)
    {
        try
        {
            var jsonBody = $"{{\"angles\":\"{requestBody}\"}}";
            var content = new StringContent(jsonBody, Encoding.UTF8, "application/json");

            // Send a POST request with the specified URI and content
            using HttpResponseMessage response = await client.PostAsync(uri, content);

            response.EnsureSuccessStatusCode();
            string responseBody = await response.Content.ReadAsStringAsync();
        }
        catch (HttpRequestException e)
        {
            text.text += "\nException " + e.InnerException.Message;
        }
    }

    public void SendControllerCoordinates(string coordinates, TMP_Text text)
    {
        this.text = text;
        var uri = robotArmManagerURI + "move"; 
        PostAsync(uri, coordinates);
    }
}