import SwiftUI
import WebKit

struct DashboardWebView: UIViewRepresentable {
    let robotIP: String
    let connectToken: UUID

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    func makeUIView(context: Context) -> WKWebView {
        let configuration = WKWebViewConfiguration()
        configuration.defaultWebpagePreferences.allowsContentJavaScript = true
        configuration.allowsInlineMediaPlayback = true

        let webView = WKWebView(frame: .zero, configuration: configuration)
        webView.scrollView.contentInsetAdjustmentBehavior = .never
        webView.isInspectable = true
        context.coordinator.webView = webView
        loadDashboard(in: webView, robotIP: robotIP)
        return webView
    }

    func updateUIView(_ webView: WKWebView, context: Context) {
        if context.coordinator.lastLoadedToken != connectToken {
            context.coordinator.lastLoadedToken = connectToken
            loadDashboard(in: webView, robotIP: robotIP)
            return
        }

        let escapedIP = robotIP.replacingOccurrences(of: "'", with: "\\'")
        let js = """
        if (document.getElementById('ipInput')) {
          document.getElementById('ipInput').value = '\(escapedIP)';
          if (typeof connect === 'function') {
            if (typeof disconnect === 'function') { disconnect(false); }
            connect();
          }
        }
        """
        webView.evaluateJavaScript(js)
    }

    private func loadDashboard(in webView: WKWebView, robotIP: String) {
        guard let baseURL = Bundle.main.resourceURL,
              let fileURL = Bundle.main.url(forResource: "dashboard", withExtension: "html")
        else {
            return
        }

        var components = URLComponents(url: fileURL, resolvingAgainstBaseURL: false)
        components?.queryItems = [URLQueryItem(name: "ip", value: robotIP)]
        let url = components?.url ?? fileURL
        webView.loadFileURL(url, allowingReadAccessTo: baseURL)
    }

    final class Coordinator {
        weak var webView: WKWebView?
        var lastLoadedToken = UUID()
    }
}
